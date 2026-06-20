# Managing Subsystem Implementations

SUAVE's managed/managing separation means any managing subsystem can be plugged in as long as it satisfies the [ROS 2 interface requirements](extend.md). This page lists all known implementations — both those shipped in this repository and external ones built by the community.

## Built-in implementations

These are included in the SUAVE repository and available out of the box.

### None (`suave_none`)

A pass-through managing subsystem that performs no adaptation. Useful as a **lower-bound baseline** when comparing adaptation strategies.

Launch: `ros2 launch suave_none suave_none.launch.py`

### Random (`suave_random`)

Applies adaptations at random intervals without analysing the system state. Useful as a **random baseline** to verify that structured adaptation adds value over chance.

Launch: `ros2 launch suave_random suave_random.launch.py`

### Metacontrol (`suave_metacontrol`)

Uses [MROS2](https://github.com/meta-control/mc_mros_reasoner), the ROS 2 implementation of the [Metacontrol](https://research.tudelft.nl/en/publications/model-based-self-awareness-patterns-for-autonomy) framework. The system is modelled in an OWL ontology ([TOMASys](https://github.com/meta-control/mc_mdl_tomasys)) with SWRL rules encoding adaptation logic. The reasoner infers which function design best satisfies the current quality attribute requirements and issues the corresponding `system_modes` reconfiguration.

Launch: `ros2 launch suave_metacontrol suave_metacontrol.launch.py`

### Behavior Tree (`suave_bt`)

Implements adaptation logic as a [BehaviorTree.CPP](https://www.behaviortree.dev/) behavior tree. Condition nodes read diagnostics and action nodes select lifecycle modes through `system_modes`. The tree can either observe behaviors started by lifecycle activation or invoke the managed behaviors through ROS 2 actions. The tree structure makes the adaptation policy explicit and easy to inspect or modify.

Launch: `ros2 launch suave_bt suave_bt.launch.py`

Enable ROS 2 action execution with:

```bash
ros2 launch suave_bt suave_bt.launch.py use_action_server:=true
```

`use_action_server` defaults to `false`. When enabled, the BT sends goals to
`spiral_search`, `follow_pipeline`, `recover_thrusters`, and
`recharge_battery`; the launch file applies the same setting to the managed
lifecycle nodes.

---

## External implementations

These repositories implement a managing subsystem for SUAVE and are maintained separately.

### ROSA (`suave_rosa`)

[Repository](https://github.com/kas-lab/rosa) · [Paper](https://www.frontiersin.org/journals/robotics-and-ai/articles/10.3389/frobt.2025.1531743/full)

ROSA is a knowledge-based self-adaptation framework for ROS 2. Its knowledge base is implemented with [TypeDB](https://typedb.com/), a strongly-typed graph database with a native inference engine. ROSA subscribes to `/diagnostics`, reasons over the knowledge base to select the appropriate adaptation, and issues reconfigures the managed system directly, by-passing `system_modes`. The `suave_rosa` repository provides the ROSA managing subsystem configured for SUAVE.

### Planta (`suave_planta`)

[Repository](https://github.com/kas-lab/suave_planta)

Planta is a PDDL-based managing subsystem. The adaptation problem is captured in an ontology based on [pddl-TOMASys](https://github.com/kas-lab/pddl_tomasys) (a modified version of the ontology used by the Metacontrol implementation), which is then automatically translated into PDDL. [PlanSys2](https://github.com/PlanSys2/ros2_planning_system) is used to plan, execute, and re-plan adaptations at runtime.

---

## Adding your own

See the [Extending SUAVE](extend.md) page for the interface requirements and launch file conventions needed to connect a new managing subsystem.
