# Embodied AI Skills

This directory contains two complementary guidance packages:

- `embodied_gas_search/`: physical single-vehicle workflow, operation, and
  evidence boundaries;
- `heterogeneous_marl/`: reproduction of the current MAPPO simulation,
  research-extension guidance, and a staged path toward physical heterogeneous
  multi-vehicle systems.

For agent-facing task routing, start with `heterogeneous_marl/SKILL.md`.

The multi-agent simulator's source of truth is `../mappo/`. Its results are
simulation-only; they do not establish multi-vehicle hardware, wireless, or
physical closed-loop validation. The physical single-car skill remains
separate and is not replaced by the MAPPO simulation.

Start with `../readme.md`, then consult the relevant skill package and the
architecture, interface, hardware, and limitation documents. Treat proposed
physical interfaces as design contracts until their implementation and tests
exist.
