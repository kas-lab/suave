# System Architecture

## Overview

SUAVE is structured around a strict **managed/managing subsystem** separation, illustrated below:

![SUAVE system overview](https://user-images.githubusercontent.com/20564040/227582678-e1494c89-1b44-4bac-aef6-bdd77127dfa6.png)

The **managed subsystem** implements the AUV's mission functionalities. The **managing subsystem** is pluggable — it observes the system through a standard monitoring interface and reconfigures the managed subsystem through standard adaptation interfaces. The two subsystems communicate exclusively through ROS 2 interfaces, so any managing subsystem implementation can be swapped in without modifying the managed side.

---

## Managed subsystem

The managed subsystem is implemented in the `suave` package. It runs on top of ArduSub/MAVROS and exposes four reconfigurable **functionalities**, each implemented as a ROS 2 lifecycle node managed by [system_modes](https://github.com/micro-ROS/system_modes).

### Functionalities and their modes

**`f_generate_search_path`** — Spiral search for the pipeline.

| Mode | Node state | Spiral altitude |
|---|---|---|
| `fd_spiral_high` | active | 3.0 m |
| `fd_spiral_medium` | active | 2.0 m |
| `fd_spiral_low` | active | 1.0 m |
| `fd_unground` | inactive | — |

**`f_follow_pipeline`** — Follow and inspect a detected pipeline.

| Mode | Node state |
|---|---|
| `fd_follow_pipeline` | active |
| `fd_unground` | inactive |

**`f_maintain_motion`** — Thruster management and recovery.

| Mode | Node state |
|---|---|
| `fd_all_thrusters` | inactive |
| `fd_recover_thrusters` | active |
| `fd_unground` | inactive |

**`generate_recharge_path`** — Battery recharge path generation (auxiliary).

| Mode | Node state |
|---|---|
| `normal` | active |
| `inactive` | inactive |

The mode configurations are defined in [`suave/config/suave_modes.yaml`](https://github.com/kas-lab/suave/blob/main/suave/config/suave_modes.yaml).

### Lifecycle and action execution

The search, inspection, recovery, and recharge lifecycle nodes support two
execution policies through `use_action_server`:

- With the default `false`, lifecycle activation starts the behavior.
- With `true`, lifecycle activation prepares the node and an accepted ROS 2
  action goal starts the behavior.

The Behavior Tree manager can use the second policy to send goals to
`spiral_search`, `follow_pipeline`, `recover_thrusters`, and
`recharge_battery`. Its launch file passes one value to both the lifecycle
nodes and the BT blackboard so the execution policy stays consistent.

---

## Monitor subsystem

The `suave_monitor` package contains three monitor nodes that continuously observe the environment or the system and report their findings on the `/diagnostics` topic using `diagnostic_msgs/DiagnosticArray`.

| Node | Observes | Published key |
|---|---|---|
| `thruster_monitor` | Simulated thruster status | `c_thruster_<N>` = `FALSE`/`RECOVERED` |
| `water_visibility_observer` | Simulated water visibility | `water_visibility` = float (m) |
| `battery_monitor` | Simulated battery state | `battery_level` = float (0–1) |

All three publish at regular intervals. The managing subsystem is expected to maintain its own internal model by subscribing to `/diagnostics`.

---

## Managing subsystem (pluggable)

The managing subsystem closes the adaptation loop. It has no prescribed internal architecture — implementations range from rule-based (BT, Metacontrol) to planning-based (Planta) to knowledge-based (ROSA). All implementations must satisfy the following interface:

### Inputs

- **`/diagnostics`** (`diagnostic_msgs/DiagnosticArray`) — monitoring data from the monitor subsystem.

### Outputs

- **`/f_generate_search_path/change_mode`** (`system_modes_msgs/srv/ChangeMode`) — change the search path generation mode.
- **`/f_follow_pipeline/change_mode`** (`system_modes_msgs/srv/ChangeMode`) — change the pipeline following mode.
- **`/f_maintain_motion/change_mode`** (`system_modes_msgs/srv/ChangeMode`) — change the motion maintenance mode.
- **`/task/request`** and **`/task/cancel`** (`suave_msgs/srv/Task`) — request or cancel high-level mission tasks.

See [Extending SUAVE](extend.md) for launch file conventions when connecting a new managing subsystem.

---

## Adaptation loop

The end-to-end adaptation cycle follows the MAPE-K pattern:

1. **Monitor** — `suave_monitor` nodes observe thruster health, water visibility, and battery level and publish findings to `/diagnostics`.
2. **Analyze** — The managing subsystem subscribes to `/diagnostics` and detects whether the system is meeting its quality requirements.
3. **Plan** — The managing subsystem selects the appropriate function design (mode) for each affected functionality.
4. **Execute** — The managing subsystem calls `change_mode` on the relevant `system_modes` service. `system_modes` transitions the lifecycle node to the target mode, which may change node parameters (e.g., spiral altitude) or activate/deactivate nodes.

The `mission_metrics` node observes this cycle and records the **reaction time** — the elapsed time between a disturbance appearing in `/diagnostics` and the corresponding lifecycle node transition completing. See the [Metrics Reference](metrics.md) for details.
