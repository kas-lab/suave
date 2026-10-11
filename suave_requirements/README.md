# suave_requirements

Runtime monitoring of the SUAVE RELAX requirements (`requirement_monitor`) and
offline plotting of their fulfillment from recorded campaign bags.

## Plotting requirement fulfillment

`scripts/plot_requirement_fulfillment.py` writes one comparison figure per
requirement topic (thruster availability, search footprint, and the combined,
thruster, water-visibility and battery reaction times) plus a
`coverage.json` provenance report. Run it with the workspace sourced:

```Bash
C=~/suave/results/batches/<batch>/campaigns/<campaign>
python3 src/suave/suave_requirements/scripts/plot_requirement_fulfillment.py \
  --bags-dir $C/rosbags --output-dir $C/requirement_fulfillment_plots
```

Options: `--managers` (default: all six exp3 managers), `--samples` (grid
points per run, default `1001`, minimum `2`), `--formats` (default `png pdf`).
The `rebetmc` manager is labelled **TacticalRetreat** in figures.

### Which bags are used

Bags are selected from the campaign folder (the parent of `--bags-dir`) with
`suave_tools.runs.accepted_runs`, never by scanning directory names:

- A trial counts only if its entry in the campaign's `runs.json`
  (`schema_version: 2`) has `"status": "completed"`.
- A completed trial must have `"bag": "accepted"`; its bag is
  `rosbags/<manager>_<mission>_<run_id>_<attempt_id>/`. A completed trial
  without an accepted bag, or an unsupported `runs.json`, is an error, not a
  silent omission.
- Trials run with recording disabled (`"bag": null`) are reported as
  `recording_disabled`.
- Bags under `rosbags/pending/` or `rosbags/failed/` are never loaded.

An unreadable selected bag is an error. A bag that is readable but lacks a
topic is allowed and reported in `coverage.json`.

### Timing and statistics

- Time is **normalized recording time**: 0–100 % of each bag's metadata
  start/duration, using the same origin for every topic. This is not mission
  time; mission boundaries are not inferred.
- Each run is resampled with previous-value hold: no interpolation and no
  lookahead. Before a topic's first message the value is unknown (gap),
  never zero. At duplicate timestamps the last message wins.
- Non-finite values, and values outside `[0, 1]` by more than `1e-6`, are
  counted as invalid samples and excluded.
- Curves are the mean over runs with a value at that grid point. Bands are
  approximate 95 % confidence intervals (`1.96 * SD / sqrt(n)`, `ddof=1`) for
  `n >= 2`; with `n = 1` only the mean is drawn. Legends show the number of
  contributing runs, and `coverage.json` holds per-grid-point counts.
- Managers with no observations for a topic are named in the figure caption
  and listed under `no_observations` in `coverage.json`; managers with no
  accepted runs at all are listed separately under `no_accepted_runs`. A
  topic with no data at all gets an explicit no-data figure.
