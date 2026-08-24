import jax
import jax.numpy as jnp
import flax.linen as nn
import numpy as np
from flax.linen.initializers import constant, orthogonal
from typing import Sequence
import distrax


def get_activation_fn(name: str):
    """ Get activation function by name from the flax.linen module."""
    try:
        # Use getattr to dynamically retrieve the activation function from jax.nn
        return getattr(nn, name)
    except AttributeError:
        raise ValueError(f"Activation function '{name}' not found. Name must be the same as in flax.linen!")


class FullyConnectedNet(nn.Module):

    hidden_layer_dims: Sequence[int]
    output_dim: int
    activation: str = "tanh"
    output_activation: str = None    # none means linear activation
    use_running_mean_stand: bool = True
    squeeze_output: bool = True

    def setup(self):
        self.activation_fn = get_activation_fn(self.activation)
        self.output_activation_fn = get_activation_fn(self.output_activation) \
            if self.output_activation is not None else lambda x: x

    @nn.compact
    def __call__(self, x):

        if self.use_running_mean_stand:
            x = RunningMeanStd()(x)

        # build network
        for i, dim_layer in enumerate(self.hidden_layer_dims):
            x = nn.Dense(dim_layer, kernel_init=orthogonal(np.sqrt(2)), bias_init=constant(0.0))(x)
            x = self.activation_fn(x)

        # add last layer
        x = nn.Dense(self.output_dim, kernel_init=orthogonal(0.01), bias_init=constant(0.0))(x)
        x = self.output_activation_fn(x)

        return jnp.squeeze(x) if self.squeeze_output else x


class ActorCritic(nn.Module):
    action_dim: Sequence[int]
    activation: str = "tanh"
    init_std: float = 1.0
    learnable_std: bool = True
    hidden_layer_dims: Sequence[int] = (1024, 512)
    actor_obs_ind: jnp.ndarray = None
    critic_obs_ind: jnp.ndarray = None

    def setup(self):
        self.activation_fn = get_activation_fn(self.activation)

    @nn.compact
    def __call__(self, x):

        x = RunningMeanStd()(x)

        # build actor
        actor_x = x if self.actor_obs_ind is None else x[..., self.actor_obs_ind]
        actor_mean = FullyConnectedNet(self.hidden_layer_dims, self.action_dim, self.activation,
                                       None, False, False)(actor_x)
        actor_logtstd = self.param("log_std", nn.initializers.constant(jnp.log(self.init_std)),
                                   (self.action_dim,))
        if not self.learnable_std:
            actor_logtstd = jax.lax.stop_gradient(actor_logtstd)

        pi = distrax.MultivariateNormalDiag(actor_mean, jnp.exp(actor_logtstd))

        # build critic
        critic_x = x if self.critic_obs_ind is None else x[..., self.critic_obs_ind]
        critic = FullyConnectedNet(self.hidden_layer_dims, 1, self.activation, None, False, False)(critic_x)

        return pi, jnp.squeeze(critic, axis=-1)


class ActorCriticWithAlpha(nn.Module):
    """Actor-Critic network with per-joint activation multipliers (Stage 1 Passive Policy).

    The actor outputs a concatenated vector [a_q | a_alpha] of size 2 * action_dim:
      - a_q:    joint position targets (tanh-bounded, same as standard ActorCritic)
      - a_alpha: per-joint activation multipliers in [0, 1] via sigmoid

    The gated torque in the environment is: tau_j = alpha_j * PD(q*, q, dq*, dq)

    The critic is identical to the standard ActorCritic critic.
    """
    action_dim: int             # number of joints (NOT doubled — network doubles internally)
    activation: str = "tanh"
    init_std: float = 1.0
    learnable_std: bool = True
    hidden_layer_dims: Sequence[int] = (1024, 512)
    actor_obs_ind: jnp.ndarray = None
    critic_obs_ind: jnp.ndarray = None
    alpha_init_bias: float = 2.0    # sigmoid(2.0) ≈ 0.88 — starts near-active, not at 0.5

    def setup(self):
        self.activation_fn = get_activation_fn(self.activation)

    @nn.compact
    def __call__(self, x):

        x = RunningMeanStd()(x)

        # ------------- ACTOR -------------
        actor_x = x if self.actor_obs_ind is None else x[..., self.actor_obs_ind]

        # Shared actor trunk
        actor_trunk = FullyConnectedNet(self.hidden_layer_dims, self.hidden_layer_dims[-1],
                                        self.activation, None, False, False)(actor_x)

        # Head 1: joint position targets a_q  (linear output, tanh handled by env action scaling)
        a_q = nn.Dense(self.action_dim, kernel_init=orthogonal(0.01), bias_init=constant(0.0))(actor_trunk)

        # Head 2: activation multipliers a_alpha in [0,1] via sigmoid
        # Initialized with positive bias so alpha starts near 0.88 (robot begins near-active)
        a_alpha_logit = nn.Dense(self.action_dim, kernel_init=orthogonal(0.01),
                                 bias_init=constant(self.alpha_init_bias))(actor_trunk)
        a_alpha = nn.sigmoid(a_alpha_logit)

        # Concatenate: combined action vector seen by env = [a_q | a_alpha]
        actor_mean = jnp.concatenate([a_q, a_alpha], axis=-1)

        # Log-std over full output (position targets only — alpha has no noise)
        actor_logtstd = self.param("log_std", nn.initializers.constant(jnp.log(self.init_std)),
                                   (self.action_dim,))
        if not self.learnable_std:
            actor_logtstd = jax.lax.stop_gradient(actor_logtstd)

        # Distribution over a_q only; a_alpha is deterministic (mean of sigmoid head)
        pi = distrax.MultivariateNormalDiag(actor_mean[..., :self.action_dim],
                                            jnp.exp(actor_logtstd))

        # Store alpha in pi for reward computation (accessed via pi.loc[action_dim:])
        # We use a wrapper to carry both a_q sample and a_alpha through the training loop.
        # Combined action = sampled a_q concatenated with deterministic a_alpha.
        pi = _AlphaWrappedDistribution(pi, a_alpha)

        # ------------- CRITIC -------------
        critic_x = x if self.critic_obs_ind is None else x[..., self.critic_obs_ind]
        critic = FullyConnectedNet(self.hidden_layer_dims, 1, self.activation, None, False, False)(critic_x)

        return pi, jnp.squeeze(critic, axis=-1)


class _AlphaWrappedDistribution:
    """Thin wrapper around a distrax distribution that appends deterministic alpha
    to every sampled action so the environment sees [a_q | a_alpha]."""

    def __init__(self, base_dist, alpha):
        self._base = base_dist
        self._alpha = alpha

    def sample(self, seed):
        a_q = self._base.sample(seed=seed)
        return jnp.concatenate([a_q, self._alpha], axis=-1)

    def log_prob(self, combined_action):
        # log_prob is only over a_q (alpha is deterministic)
        n = self._base.loc.shape[-1]
        a_q = combined_action[..., :n]
        return self._base.log_prob(a_q)

    def entropy(self):
        return self._base.entropy()

    @property
    def loc(self):
        return jnp.concatenate([self._base.loc, self._alpha], axis=-1)


class RunningMeanStd(nn.Module):
    """Layer that maintains running mean and variance for input normalization."""

    @nn.compact
    def __call__(self, x):

        x = jnp.atleast_2d(x)

        # Initialize running mean, variance, and count
        mean = self.variable('run_stats', 'mean', lambda: jnp.zeros(x.shape[-1]))
        var = self.variable('run_stats', 'var', lambda: jnp.ones(x.shape[-1]))
        count = self.variable('run_stats', 'count', lambda: jnp.array(1e-6))

        # Compute batch mean and variance
        batch_mean = jnp.mean(x, axis=0)
        batch_var = jnp.var(x, axis=0) + 1e-6  # Add epsilon for numerical stability
        batch_count = x.shape[0]

        # Update counts
        updated_count = count.value + batch_count

        # Numerically stable mean and variance update
        delta = batch_mean - mean.value
        new_mean = mean.value + delta * batch_count / updated_count

        # Compute the new variance using Welford's method
        m_a = var.value * count.value
        m_b = batch_var * batch_count
        M2 = m_a + m_b + jnp.square(delta) * count.value * batch_count / updated_count
        new_var = M2 / updated_count

        # Normalize input
        normalized_x = (x - new_mean) / jnp.sqrt(new_var + 1e-8)

        # Update state variables
        mean.value = new_mean
        var.value = new_var
        count.value = updated_count

        return jnp.squeeze(normalized_x)
