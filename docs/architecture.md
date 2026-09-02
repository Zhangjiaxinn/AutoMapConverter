# Architecture

```text
user -> CLI / future GUI / Python API -> conversion workflows
                                      -> diagnostics and visualization hooks
```

The public boundary is `automap_converter.api`. The CLI and future GUI only
call this API; neither owns separate conversion logic.

```text
src/automap_converter/
  formats/                 # parsers, data models, format adapters
  conversion/
    lanelet2_to_opendrive/ # direct road, lane, junction, signal and object construction
    lanelet2_to_osm/       # direct, intentionally lossy OSM export
    opendrive_to_commonroad/ # OpenDRIVE parser and intermediate lane-network construction
    osm_to_commonroad/       # OSM graph and intermediate road-network construction
    commonroad_to_lanelet2/  # CommonRoad scenario writer for Lanelet2
  validation/              # precheck, per-workflow Stage 1/2, round trip
  visualization/           # optional viewer launches
  api/                     # public Python workflow functions
  gui/                     # reserved shell; no conversion logic here
```

The five directories above are the only atomic conversion stages. The public
API composes `OpenDRIVE -> CommonRoad -> Lanelet2` and
`OSM -> CommonRoad -> Lanelet2`; OpenDRIVE/OSM conversion is then explicitly
composed through Lanelet2. No duplicate conversion wrapper owns hidden rules.

The diagnostics package has three public direction profiles for the workflows
that share common report machinery. `_workflow_runtime.py` executes Stage 1,
batch processing and viewer checks; `_semantic_engine.py` collects Stage 2
geometry, topology and semantic statistics. Their leading underscore marks
them as internal implementation modules, not user-facing APIs.
