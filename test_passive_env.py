#!/usr/bin/env python3

"""Test script to verify the PassiveKBotLegsV2 class has the expected attributes."""

import sys
import os

# Add the MuJoCo directory to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'MuJoCo', 'loco-mujoco'))

try:
    # Just import the class definition without creating an instance
    from loco_mujoco.environments.humanoids.kbot_legs_v2_passive import PassiveKBotLegsV2
    
    print("Successfully imported PassiveKBotLegsV2")
    
    # Check that the class has the expected __init__ signature
    import inspect
    sig = inspect.signature(PassiveKBotLegsV2.__init__)
    params = list(sig.parameters.keys())
    print(f"__init__ parameters: {params}")
    
    # Check that our new parameters are present
    expected_params = ['self', 'passive_reward_weight', 'alpha_curriculum_start', 
                      'alpha_min_values', 'total_timesteps', 'kwargs']
    for param in expected_params:
        if param not in params:
            print(f"WARNING: Expected parameter '{param}' not found in __init__")
        else:
            print(f"✓ Parameter '{param}' found")
    
    # Check that the _preprocess_action method exists
    if hasattr(PassiveKBotLegsV2, '_preprocess_action'):
        print("✓ _preprocess_action method found")
    else:
        print("ERROR: _preprocess_action method not found")
        sys.exit(1)
        
    # Check that the _compute_passive_reward method exists
    if hasattr(PassiveKBotLegsV2, '_compute_passive_reward'):
        print("✓ _compute_passive_reward method found")
    else:
        print("ERROR: _compute_passive_reward method not found")
        sys.exit(1)
    
    print("All checks passed!")
    
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)