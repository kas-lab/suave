# suave_requirements

SUAVE's quality requirements, written in RELAX. The `requirement_monitor` node
computes how well each requirement is fulfilled while the mission runs, and
the scripts in `scripts/` plot that fulfillment from recorded campaign bags.

## Requirements

RELAX requirements are not simply passed or failed. Each one has a
*fulfillment degree* in `[0, 1]`, given by a fuzzy membership function over a
measured value: 1 means fully satisfied, 0 means violated.

| Requirement | RELAX expression | Measured value | Fulfillment (defaults) |
|---|---|---|---|
| Thrusters availability | `SHALL (AS CLOSE AS POSSIBLE TO (0, operational_thrusters_delta, FuzzyLeftShoulder(foot=1.0, shoulder=0.0)))` | `required_thrusters - operational_thrusters` | 1 with no missing thruster, 0 with one or more missing |
| Search footprint | `SHALL (AS MANY AS POSSIBLE (search_footprint_area, FuzzyRightShoulder(foot=0.0, shoulder=12.0)))` | camera `coverage_area` (m²) while searching | 0 at 0 m², rising linearly to 1 at 12 m² |
| Adaptation reaction time | `SHALL (AFTER e (AS EARLY AS POSSIBLE (reaction_time, FuzzyExponentialDecay(exponent=0.2))))` | seconds from a perturbation to its adaptation | `e^(-0.2 t)`: 0.37 at 5 s, 0.14 at 10 s |

**Thrusters availability.** *The AUV SHALL keep the number of operational
thrusters AS CLOSE AS POSSIBLE TO the number required by the active motion
controller.* When thrusters are missing, the controller commands thrusters
that do not respond, and the AUV deviates from its path. Recovering the
thrusters satisfies the requirement, and so would switching to a controller
that needs fewer thrusters.

**Search footprint.** *The AUV SHALL observe AS MUCH of the search area AS
POSSIBLE while searching for the pipeline.* A higher search altitude gives a
larger camera footprint, but the pipeline cannot be detected when the
altitude is above the water visibility. The search-altitude tactics manage
this trade-off. The requirement only applies until the pipeline is detected;
after that its fulfillment holds its last value.

**Adaptation reaction time.** *The AUV SHALL complete an appropriate
adaptation AS EARLY AS POSSIBLE AFTER a thruster failure, a water-visibility
change, or a low battery.* Fulfillment is reported per perturbation type
(`thruster`, `water_visibility`, `battery`) and as a combined value: the
moving average over the last `reaction_time_window_size` reactions of any
type.

## Runtime monitor

`suave_base.launch.py` starts `requirement_monitor`, so every SUAVE mission
publishes fulfillment. To run it on its own:

```bash
ros2 run suave_requirements requirement_monitor
```

It reads `operational_thrusters` and `coverage_area` from `/diagnostics` and
reaction times from `mission_metrics/reaction_time`, and publishes
`std_msgs/Float32` values:

| Topic | Requirement |
|---|---|
| `/requirements/thruster_availability/fulfillment` | Thrusters availability |
| `/requirements/search_footprint/fulfillment` | Search footprint |
| `/requirements/adaptation_reaction_time/fulfillment` | Reaction time, combined |
| `/requirements/adaptation_reaction_time/<type>/fulfillment` | Reaction time for `<type>` = `thruster`, `water_visibility`, `battery` |

A topic stays silent until its first input arrives. After that, values are
published on change and republished every `publishing_period`.

| Parameter | Default | Meaning |
|---|---|---|
| `required_thrusters` | `6` | Thrusters required by the motion controller |
| `thruster_availability_foot` / `_shoulder` | `1.0` / `0.0` | Left shoulder over missing thrusters |
| `search_footprint_foot` / `_shoulder` | `0.0` / `12.0` | Right shoulder over coverage area (m²) |
| `reaction_time_exponent` | `0.2` | Exponential decay rate (1/s) |
| `reaction_time_window_size` | `5` | Reactions averaged into the combined value |
| `publishing_period` | `1.0` | Republishing period (s) |

## Offline analysis

The scripts read the bags recorded in a campaign. Run them from the SUAVE
repository root in the sourced SUAVE container. Their path argument can be a
batch, its `campaigns/` folder, its `state.json`, or one campaign folder. Output goes to
`<batch>/campaign_results/<campaign>/…` unless `--output-dir` is given. Each
script's `--help` explains which bags it uses and how it samples them.

**Fulfillment across managers:** one figure per requirement, comparing the
mean fulfillment of each manager over its runs, with 95 % confidence bands.

```bash
python3 suave_requirements/scripts/plot_requirement_fulfillment.py \
  ~/suave/results/batches/<batch>
```

**Representative run:** the fulfillment, water visibility, adaptations, and
mission phases of one run (by default the most typical `rebetmc` run), as
PGFPlots tables and a standalone LaTeX figure.

```bash
python3 suave_requirements/scripts/export_representative_run_tikz.py \
  ~/suave/results/batches/<batch>
python3 suave_requirements/scripts/export_representative_run_tikz.py \
  ~/suave/results/batches/<batch> --run-id run_0_24 --xmax 150 --compile
```

**Other exporters:** `export_requirement_fulfillment_tikz.py` writes the
across-manager means as PGFPlots tables, and
`export_search_footprint_alternatives.py` exports alternative summaries of the
search footprint (censored at pipeline detection and accumulated over the search).

### Compiling the figures

The SUAVE containers do not include LaTeX: the scripts still write the
PGFPlots tables and `.tex` files, but `--compile` is skipped. To compile a
figure, run `pdflatex` on its `.tex` file on a machine with LaTeX installed,
including `pgfplots` and the `standalone` class (on Ubuntu,
`texlive-latex-extra`).
