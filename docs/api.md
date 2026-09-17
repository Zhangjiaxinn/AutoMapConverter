# Python API

```python
from automap_converter import convert, convert_and_diagnose

convert("source.osm", "target.xodr", "lanelet2", "opendrive")

result = convert_and_diagnose(
    "source.xodr",
    "target.osm",
    "opendrive",
    "lanelet2",
)
print(result.diagnostics_report)

raster = convert_and_diagnose(
    "source.osm",
    "semantic_map.tif",
    "lanelet2",
    "raster",
)
print(raster.artifacts["occupancy"])
```

Supported pairs are `lanelet2 -> opendrive`, `opendrive -> lanelet2`,
`osm -> lanelet2`, `lanelet2 -> osm`, `osm -> opendrive`,
`opendrive -> osm`, and `lanelet2 -> raster`. The two OSM/OpenDRIVE routes are
composed workflows; their diagnostic directory retains both component reports
and a composed-report index.

Pass `config_path="configs/default.yaml"` to the Python API, or `--config` to
the CLI, to control Lanelet2 routing enrichment, OSM sublayers, OpenDRIVE
version, raster resolution/supersampling, optional viewer launch, and the
diagnostic groups.
`configs/default.yaml` automatically uses its sibling `validation.yaml`; a
custom YAML file can contain both sections in one place.
