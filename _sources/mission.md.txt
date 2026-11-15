# Mission and uncertainties

SUAVE simulates an autonomous underwater vehicle (AUV), a BlueROV2 running
ArduSub, that must find and inspect a pipeline on the seabed. The AUV starts
close to where the pipeline is expected to be, but does not know its exact
location, as in a real deployment.

## Mission

The mission is a sequence of tasks:

1. **A1: Search for the pipeline.** The AUV follows a spiral path at a
   configurable altitude until its camera detects the pipeline.
2. **A2: Follow and inspect the pipeline.** Once the pipeline is detected, the
   AUV follows it and inspects as much of it as possible.

The [ROSA](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2025.1531743/full)
extension adds a third task:

3. **A3: Recharge the battery.** When the battery runs low, the AUV interrupts
   its current task, recharges, and then resumes the mission.

The default mission is time-constrained: it ends after a time limit
(`time_limit`, 300 s by default), whether or not the pipeline was found.

## Uncertainties

While the AUV operates, its environment and its own components change in
ways that are not known at design time:

| Uncertainty | Effect | Adaptation |
|---|---|---|
| **U1: Thruster failures** | One or more thrusters stop working during A1 or A2. The motion controller still commands them, so the AUV deviates from its path | Activate thruster recovery (`f_maintain_motion` in `fd_recover_thrusters`). Recovery always succeeds |
| **U2: Changing water visibility** | During A1, the camera cannot detect the pipeline when the AUV searches above the water visibility | Select a lower or higher spiral altitude (1, 2, or 3 m) for the current visibility |
| **U3: Low battery** | During A1 or A2, the battery level reaches a critical threshold | Interrupt the current task and run A3, which needs its own architectural configuration |

U1 and U2 only require changing the architecture, and the task sequence stays
the same. U3 also changes the task sequence. The scenario with U1 and U2 is
the **original use case**, and the scenario with all three is the **extended
use case**. Water current is available as an optional environmental
disturbance (`enable_water_current`), with no built-in adaptation.

The disturbances are simulated and configurable:

- **Thruster failures** are scheduled through the `thruster_events` parameter
  of `thruster_monitor`. Each entry `(thruster, failure|recovery, delay_s)`
  names a thruster (1 to 6) and the seconds since the previous event.
- **Water visibility** varies periodically between a minimum and a maximum,
  by default from 1.25 m to 3.75 m with an 80 s period.
- **Battery level** drops linearly over `discharge_time` (200 s by default).
  A level below `battery_limit` (0.25) counts as low battery.

The experiment runner can shift these disturbances randomly between runs. See
{doc}`run` for the campaign parameters.

## Search altitude trade-off

A higher search altitude gives the camera a larger footprint, so the AUV
covers the search area faster. But the pipeline is only detected when the
altitude is below the water visibility. The expected altitude for each
visibility is:

| Water visibility | Spiral altitude | Mode |
|---|---|---|
| ≥ 3.25 m | 3.0 m | `fd_spiral_high` |
| ≥ 2.25 m and < 3.25 m | 2.0 m | `fd_spiral_medium` |
| ≥ 1.25 m and < 2.25 m | 1.0 m | `fd_spiral_low` |

The thresholds are the `water_visibility_threshold` and `expected_altitude`
parameters of `mission_metrics`, which uses them to measure reaction times.

## Key performance indicators

The mission's success is measured with two KPIs:

- **Search time:** how long the AUV takes to find the pipeline.
- **Distance inspected:** how much of the pipeline the AUV inspects before the
  time limit.

SUAVE also measures the **adaptation reaction time**: how long the managing
subsystem takes to adapt after a disturbance. See {doc}`metrics` for how
each metric is recorded.

## Quality requirements

SUAVE's quality requirements are written in RELAX: each one has a fulfillment
degree between 0 (violated) and 1 (fully satisfied), instead of simply passing
or failing.

| Requirement | Statement |
|---|---|
| **Thrusters availability** | The AUV SHALL keep the number of operational thrusters AS CLOSE AS POSSIBLE TO the number required by the active motion controller |
| **Search footprint** | The AUV SHALL observe AS MUCH of the search area AS POSSIBLE while searching for the pipeline |
| **Adaptation reaction time** | The AUV SHALL complete an appropriate adaptation AS EARLY AS POSSIBLE AFTER a thruster failure, a water visibility change, or a low battery |

Each requirement captures a stakeholder concern without prescribing a
solution. For example, thrusters availability can be satisfied by recovering
the failed thrusters, or by switching to a motion controller that needs fewer
thrusters.

The `requirement_monitor` node publishes the fulfillment of each requirement
at runtime on `/requirements/<requirement>/fulfillment`. Managing subsystems
can use these topics to decide when to adapt, and the offline scripts plot
them per managing subsystem. The RELAX expressions, fuzzy membership
functions, topics, and scripts are described in the
[suave_requirements README](https://github.com/kas-lab/suave/blob/main/suave_requirements/README.md).
