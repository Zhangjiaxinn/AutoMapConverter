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
```

Supported pairs are `lanelet2 -> opendrive`, `opendrive -> lanelet2`,
`osm -> lanelet2`, `lanelet2 -> osm`, `osm -> opendrive`, and
`opendrive -> osm`. The latter two are explicit two-stage workflows; their
diagnostic directory retains both stage reports and a composed-report index.

Pass `config_path="configs/default.yaml"` to the Python API, or `--config` to
the CLI, to control Lanelet2 routing enrichment, OSM sublayers, OpenDRIVE
version, optional viewer launch, and the three Stage 2 diagnostic groups.
`configs/default.yaml` automatically uses its sibling `validation.yaml`; a
custom YAML file can contain both sections in one place.
