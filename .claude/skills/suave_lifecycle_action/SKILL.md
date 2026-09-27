---
name: suave-lifecycle-action
description: Use when modifying SUAVE lifecycle nodes, ROS action-server behavior, managed behaviors, cancellation/deactivation handling, or related action tests.
---

# SUAVE Lifecycle + Action Skill

Use this skill for changes to SUAVE managed-behavior nodes such as:

- `src/suave/suave/suave/spiral_search_lc.py`
- `src/suave/suave/suave/follow_pipeline_lc.py`
- `src/suave/suave/suave/recharge_battery_lc.py`
- `src/suave/suave/suave/recover_thrusters_lc.py`
- shared helpers such as `action_server_utils.py`, `ros_service_utils.py`, and `mavros_position_controller.py`
- tests under `src/suave/suave/test/test_*_lc.py`

## Required context

Before editing:

1. Read `src/suave/AGENTS.md`.
2. Check status and preserve unrelated changes:
   ```bash
   git -C src/suave status --short
   ```
3. Inspect the node and the corresponding test file before changing behavior.

## Lifecycle/action design rules

- Lifecycle action servers are created in `on_configure()` so they remain discoverable.
- The `use_action_server` parameter defaults to `False`; legacy lifecycle activation behavior starts work when false, while action mode waits for an accepted goal.
- Use shared helpers from `suave/suave/action_server_utils.py` instead of duplicating goal acceptance, cancellation, lifecycle-state, or mode checks.
- Use `call_service_with_timeout()` from `suave/suave/ros_service_utils.py` for bounded service waits.
- Keep action callbacks as policy wrappers around node-local core behavior. Core routines should return explicit outcomes; action wrappers map them to `succeed()`, `abort()`, or `canceled()`.
- Do not leave legacy timers or tasks running in action mode.

## Cancellation and lifecycle stops

- Lifecycle nodes use `_abort_event` and `_goal_executing` patterns. Preserve them unless intentionally refactoring the whole node.
- Clear `_abort_event` on activation and set it on deactivation/shutdown.
- Wait for active action execution to finish before destroying resources it may use.
- Pass cancellation as a callable, for example `lambda: goal_handle.is_cancel_requested`; do not pass a captured boolean snapshot.
- Represent lifecycle deactivation separately from client cancellation when a node tracks stop reasons.
- For long-running operations, inject a stop policy and check it inside every loop, service wait, and setpoint wait.

## MAVROS and threading conventions

- `MavrosPositionController` is not a separately spun child node; it creates ROS entities through the owning lifecycle node.
- Keep local-position subscriptions in callback groups that cannot be blocked by behavior callbacks.
- Use `MultiThreadedExecutor` for nodes with blocking behavior/action callbacks and concurrent subscriptions.
- On ROS 2 Humble, `Node.create_rate()` returns `rclpy.timer.Rate`.

## Testing conventions

- Action-server tests should spin helper nodes and nodes under test in separate executors via `suave/test/action_test_utils.py`.
- Test terminal-state mapping and cancellation/deactivation inside inner wait loops, not only goal acceptance.
- Mock goal handles need `is_cancel_requested = False` when callbacks wrap that property in a lambda.
- Monkeypatched core methods that accept `cancel_requested=None` should preserve that argument.
- Treat `rclpy.shutdown() has been called` after passing tests as known stderr noise only when pytest/colcon report success.

## Validation

Run focused checks inside the SUAVE container when practical:

```bash
docker exec suave bash -lc 'cd /home/ubuntu-user/suave_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && python3 -m py_compile <changed.py> && python3 -m flake8 <changed.py> && python3 -m pydocstyle <changed.py>'
```

For lifecycle/action changes, run the package tests:

```bash
docker exec suave bash -lc 'cd /home/ubuntu-user/suave_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && colcon test --packages-select suave --event-handlers console_direct+'
```

If only one focused pytest is needed during iteration:

```bash
docker exec suave bash -lc 'cd /home/ubuntu-user/suave_ws && source /opt/ros/humble/setup.bash && source install/setup.bash && python3 -m pytest -q src/suave/suave/test/test_<node>.py'
```
