# Sim plans

Active plans for in-flight sim work. A plan is deleted once its feature ships
(the design record then lives in [`../journal/shipped/`](../journal/shipped/)).

- [`sim-fidelity/`](sim-fidelity/README.md) — the phased sim-fidelity plan:
  per-subsystem implementation plans (wind & turbulence, atmosphere/aero,
  magnetic field, power model, sensor error models).
- [`sim-mockup-parity-gaps.md`](sim-mockup-parity-gaps.md) — the sim features the
  UI mockup specifies that the real `SimulatorWidget` + physics don't implement
  yet, with prioritisation feeding the fidelity plan.

See the component overview at [`../README.md`](../README.md).
