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
    lanelet2_to_raster/    # relation-aware semantic and occupancy rasterization
    opendrive_to_commonroad/ # OpenDRIVE parser and intermediate lane-network construction
    osm_to_commonroad/       # OSM graph and intermediate road-network construction
    commonroad_to_lanelet2/  # CommonRoad scenario writer for Lanelet2
  validation/              # source checks, per-workflow and round-trip diagnostics
  visualization/           # optional viewer launches
  api/                     # public Python workflow functions
  gui/                     # reserved shell; no conversion logic here
```

The six directories above are the reusable conversion modules. The public
API composes `OpenDRIVE -> CommonRoad -> Lanelet2` and
`OSM -> CommonRoad -> Lanelet2`; OpenDRIVE/OSM conversion is then explicitly
composed through Lanelet2. Lanelet2 -> Raster directly reconstructs lanelet
polygons and centerlines before supersampled grid encoding. No duplicate
conversion wrapper owns hidden rules.

The diagnostics package has one profile per atomic public direction.
`_workflow_runtime.py` coordinates source checks, conversion, target loading
and batch processing; `_semantic_engine.py` collects vector quality metrics and
dispatches raster product diagnostics. Their leading underscore marks them as
internal implementation modules, not user-facing APIs.

## Lanelet2 Raster Pipeline

![Lanelet2 to raster pipeline](assets/lanelet2_to_raster_pipeline.png)

The raster path parses `type=lanelet` relations, resolves left/right boundary
ways, aligns their orientation and derives a centerline when one is not
explicitly provided. It uses `local_x/local_y` when complete and otherwise
projects WGS84 coordinates into the local UTM zone. Supersampled rasterization
reduces aliasing while a JSON sidecar preserves identities, regulatory
references, successors/predecessors and lateral adjacency that cannot be
represented faithfully by pixels alone.
