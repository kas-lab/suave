# SUAVE: An Exemplar for Self-Adaptive Underwater Vehicles

[![GitHub release (latest by date)](https://img.shields.io/github/v/release/kas-lab/suave)](https://github.com/kas-lab/suave/releases)
[![DOI](https://zenodo.org/badge/530141277.svg)](https://zenodo.org/badge/latestdoi/530141277)
[![Main](https://github.com/kas-lab/suave/actions/workflows/main.yml/badge.svg?branch=main)](https://github.com/kas-lab/suave/actions/workflows/main.yml)
[![Docker Images](https://github.com/kas-lab/suave/actions/workflows/container.yml/badge.svg)](https://github.com/kas-lab/suave/actions/workflows/container.yml)
[![Documentation](https://github.com/kas-lab/suave/actions/workflows/doc.yml/badge.svg)](https://kas-lab.github.io/suave/)

SUAVE (Self-adaptive Underwater Autonomous Vehicle Exemplar) is a ROS 2 Humble
exemplar for self-adaptive robotics. An autonomous underwater vehicle (AUV)
searches for a pipeline on the seabed, then follows and inspects it, while its
thrusters fail, the water visibility changes, and its battery drains.

SUAVE separates the **managed subsystem**, which implements the AUV's mission,
from the **managing subsystem**, which implements the adaptation logic. Any
managing subsystem that uses SUAVE's ROS 2 interfaces can be plugged in, so
different adaptation approaches can be compared on the same missions and
metrics.

**Documentation: <https://kas-lab.github.io/suave/>**

A video of SUAVE running (click the image to open it):

<p align="center">
  <a href="https://www.youtube.com/watch?v=X8yUZjM5bfk">
    <img src="docs/images/image.png" width="500">
  </a>
</p>

An overview of the system:

<p align="center">
  <img src="https://user-images.githubusercontent.com/20564040/227582678-e1494c89-1b44-4bac-aef6-bdd77127dfa6.png" width="500">
</p>

## Quick start

With [Docker](https://docs.docker.com/get-docker/) installed:

```bash
git clone https://github.com/kas-lab/suave.git
cd suave
source env.sh
suave docker run
suave run
```

`suave docker run` pulls the headless image and starts a container named
`suave`, and `suave run` starts an experiment campaign in it. Results are
saved to `~/suave/results` on the host. Add `--gpu none` to `suave docker run`
on machines without an NVIDIA GPU.

To try SUAVE in a browser-based desktop instead, or to install it without
Docker, see [Docker](https://kas-lab.github.io/suave/docker.html) and
[Installation](https://kas-lab.github.io/suave/installation.html).

## Documentation

| Page | Contents |
|---|---|
| [Mission and uncertainties](https://kas-lab.github.io/suave/mission.html) | What the AUV does, the uncertainties it faces, its KPIs and quality requirements |
| [Installation](https://kas-lab.github.io/suave/installation.html) | Local installation with Gazebo, ROS 2, ArduSub, and the SUAVE workspace |
| [Docker](https://kas-lab.github.io/suave/docker.html) | Headless and browser-based images |
| [SUAVE CLI](https://kas-lab.github.io/suave/cli.html) | The `suave` command for containers, builds, tests, and experiments |
| [Run SUAVE](https://kas-lab.github.io/suave/run.html) | Campaigns, batches, failure diagnosis, and recovery |
| [Processing results](https://kas-lab.github.io/suave/results.html) | Summaries, statistical tests, plots, and LaTeX tables |
| [Metrics](https://kas-lab.github.io/suave/metrics.html) | Recorded mission metrics and reaction times |
| [Architecture](https://kas-lab.github.io/suave/architecture.html) | Managed functionalities, monitors, and managing-subsystem interfaces |
| [Extending SUAVE](https://kas-lab.github.io/suave/extend.html) | Connecting a new managing subsystem or adding functionality |
| [Implementations](https://kas-lab.github.io/suave/implementations.html) | Available managing subsystems |
| [Troubleshooting](https://kas-lab.github.io/suave/troubleshooting.html) | Common problems and dependency versions |

## Managing subsystems

Built into this repository:

- **None:** no adaptation, a lower-bound baseline.
- **Random:** adapts at random intervals, a chance baseline.
- **Metacontrol:** ontology-based reasoning with [MROS2](https://github.com/meta-control/mc_mros_reasoner).
- **Behavior Tree:** adaptation logic as a [BehaviorTree.CPP](https://www.behaviortree.dev/) tree.

Maintained separately:

- **[ROSA](https://github.com/kas-lab/rosa):** knowledge-based adaptation with TypeDB.
- **[Planta](https://github.com/kas-lab/suave_planta):** PDDL planning with PlanSys2.

See [Implementations](https://kas-lab.github.io/suave/implementations.html)
for details and [Extending SUAVE](https://kas-lab.github.io/suave/extend.html)
to connect your own.

## Repository layout

| Path | Contents |
|---|---|
| [`suave`](suave) | Managed subsystem functionalities and simulation launch |
| [`suave_monitor`](suave_monitor) | Thruster, water visibility, and battery monitors |
| [`suave_missions`](suave_missions) | Mission nodes and mission configuration |
| [`suave_base`](suave_base) | Reusable launch of the managed system and metrics, for external managing subsystems |
| [`suave_bringup`](suave_bringup) | Launch composition of missions with the built-in managing subsystems |
| [`suave_metrics`](suave_metrics) | Mission metrics collection |
| [`suave_requirements`](suave_requirements) | Runtime fulfillment of the quality requirements and its plots |
| [`suave_runner`](suave_runner) | Experiment runner and statistical analysis |
| [`suave_tools`](suave_tools) | Shared helpers for result layout and labels |
| [`suave_cli`](suave_cli) | The `suave` command-line tool |
| [`suave_msgs`](suave_msgs) | SUAVE messages, services, and actions |
| [`suave_managing`](suave_managing) | Built-in managing subsystems: `suave_none`, `suave_random`, `suave_metacontrol`, `suave_bt` |
| [`docker`](docker) | Dockerfiles and image scripts |
| [`docs`](docs) | Source of the documentation site |
| [`runner`](runner) | Legacy shell runner scripts |

## Citation

A paper describing SUAVE was presented at the SEAMS 2023 artifact track
([IEEE](https://ieeexplore.ieee.org/abstract/document/10173938),
[arXiv pre-print](https://arxiv.org/abs/2303.09220)). If you find this
repository useful, please consider citing it:

```bibtex
@INPROCEEDINGS{10173938,
  author={Silva, Gustavo Rezende and Päßler, Juliane and Zwanepol, Jeroen and Alberts, Elvin and Tarifa, S. Lizeth Tapia and Gerostathopoulos, Ilias and Johnsen, Einar Broch and Corbato, Carlos Hernández},
  booktitle={2023 IEEE/ACM 18th Symposium on Software Engineering for Adaptive and Self-Managing Systems (SEAMS)},
  title={SUAVE: An Exemplar for Self-Adaptive Underwater Vehicles},
  year={2023},
  volume={},
  number={},
  pages={181-187},
  doi={10.1109/SEAMS59076.2023.00031}}
```

To reproduce the paper's artifact, see
[Reproduce the SEAMS 2023 artifact](https://kas-lab.github.io/suave/installation.html#reproduce-the-seams-2023-artifact).

## Acknowledgments

This project has received funding from the European Union’s Horizon 2020
research and innovation programme under the Marie Skłodowska-Curie grant
agreement No. 956200.

Please visit [our website](https://remaro.eu/) for more information on our project.

![REMARO Logo](https://remaro.eu/wp-content/uploads/2020/09/remaro1-right-1024.png)
