# Design Decisions — 96V GaN Humanoid Leg Drive

> **How to use:** Log every significant design decision here — topology choices, component selections,
> control strategy picks, or architectural trade-offs. Include why alternatives were rejected.
> This becomes your thesis justification trail.

---

## Decision Template

```
## DD-XXX: <Title>

**Date:** YYYY-MM-DD
**Phase:** phaseX
**Jira:** KAN-XX
**Decision:** <What was decided>
**Alternatives considered:** <What else was evaluated and why rejected>
**Rationale:** <Engineering justification>
**Impact:** <What this affects downstream>
```

---

## Decisions

<!-- Most recent at top -->

## DD-001: Phase-based repo structure

**Date:** 2026-04-02
**Phase:** All
**Jira:** N/A
**Decision:** Organize repo by project phases (phase0–4) matching the Gantt timeline, not by domain (matlab/, psim/, etc.)
**Alternatives considered:** Domain-first structure (all MATLAB together, all PSIM together across phases)
**Rationale:** Phase-first keeps all work for a given milestone collocated, easier for thesis chapter writing and milestone handoffs
**Impact:** All future folders, branches, and data_raw entries should follow the phaseX prefix convention
