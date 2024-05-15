
# Change Log
All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](http://keepachangelog.com/)
and this project adheres to [Semantic Versioning](http://semver.org/).

## Unreleased

### Added

1. Added battery monitor node

2. Added recharge battery task

4. Added battery_constraint argument to mission [PR #155]

5. Added qa_comparison_operator to water_visibility in suave.owl

6. Documentation with sphinx

7. CI to build sphinx documentation

8. Added mission metrics node

9. Add a behavior tree as a managing subsystem for SUAVE and SUAVE extended to serve as a baseline for comparison

### Changed

1. Ardusub, mavros, ros_gz, mc_mros_reasoner, mc_mdl_tomasys, mros_ontology verions

2. Removed unused code

3. Mission config default parameters

4. Refactored repository. Created new packages : suave_monitor, and suave_metrics. Moved suave_metacontrol under suave_managing

### Fixed

1. README.md

2. Fix task bridge callbacks

## 1.2.1 [SEAMS Publication]

### Fixed

1. Minor bugs

## 1.2.0 [SEAMS Publication]

Getting repository camera-ready

### Added

1. Added github action to build the docker images automatically, and push it to the repository registry

### Changed

1. Refactor mission and mission bridge to match the paper description
  * [Issue 117](https://github.com/kas-lab/suave/issues/117)


2. Upgraded mavros, mavros_wrapper, and mros versions

3. Refactor dockerfiles

4. Moved files within the repository

5. Updated README

### Fixed
