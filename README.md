# AutoMapConverter

AutoMapConverter is a research-oriented vector-map conversion toolkit for
autonomous-driving applications. It provides explicit, inspectable conversion
workflows for Lanelet2, OpenDRIVE and OpenStreetMap (OSM), together with
traceable quality diagnostics for geometry, topology and traffic semantics.

## Supported Workflows

| Source | Target | Internal workflow |
| --- | --- | --- |
| Lanelet2 | OpenDRIVE | direct |
| OpenDRIVE | Lanelet2 | OpenDRIVE -> intermediate lane-network model -> Lanelet2 |
| OSM | Lanelet2 | OSM -> intermediate road-graph model -> Lanelet2 |
| Lanelet2 | OSM | direct, intentionally lossy |
| OSM | OpenDRIVE | OSM -> Lanelet2 -> OpenDRIVE |
| OpenDRIVE | OSM | OpenDRIVE -> Lanelet2 -> OSM |

The two composed routes reuse the explicit Lanelet2 workflows rather than
maintaining duplicate direct implementations. This keeps every conversion rule
inspectable and auditable.

## Installation

```bash
git clone https://github.com/REPLACE_WITH_OWNER/AutoMapConverter.git
cd AutoMapConverter
python -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
```

For a fully reproducible Linux runtime, including Lanelet2, `esmini`
(`odrviewer` and `odrplot`), ASAM OpenDRIVE Quality Checker and `osmium-tool`,
use Docker:

```bash
docker build -t automap-converter:ci .
docker run --rm -it -v "$PWD:/workspace" automap-converter:ci bash
```

The native Python installation includes Lanelet2 1.2.2 on supported Linux
Python 3.10-3.11 environments. RViz, JOSM and QGIS remain optional desktop
applications for manual visual comparison.

## Quick Start

```bash
automap-converter convert --from lanelet2 --to opendrive \
  --input input_map.osm --output output_map.xodr --diagnose
```

Python API:

```python
from automap_converter import convert_and_diagnose

result = convert_and_diagnose(
    "input_map.osm",
    "output_map.xodr",
    source_format="lanelet2",
    target_format="opendrive",
)
print(result.diagnostics_report)
```

## Diagnostics

- **Stage 1:** source legality, target parsing/loading, and available viewer checks.
- **Stage 2:** geometry, topology, references, and source-element mapping.
- **Round trip:** optional A -> B -> A' comparison for geometry, topology, and semantics.

Reports are written as readable `.txt` and structured `.json`. When configured,
the raw ASAM OpenDRIVE Quality Checker `.xqar` result is retained beside them.

## Layout

```text
src/automap_converter/
  formats/                 # Lanelet2/OpenDRIVE/OSM adapters and models
  conversion/              # five atomic conversion stages
    lanelet2_to_opendrive/
    lanelet2_to_osm/
    opendrive_to_commonroad/
    osm_to_commonroad/
    commonroad_to_lanelet2/
  validation/              # precheck, Stage 1, Stage 2, round-trip checks
  api/                     # Python workflow API
  visualization/           # viewer integrations
  gui/                     # reserved GUI shell, calls API only
```

`data/samples/` contains compact runnable inputs. `data/results/` is the
standard destination for converted maps and their diagnostic reports.

Public YAML profiles in `configs/` control Lanelet2 topology enrichment, OSM
sublayer extraction, the OpenDRIVE output version, viewer launch behavior and
the three Stage 2 diagnostic groups. Use `--config configs/default.yaml` to
select a profile explicitly.

See [docs/architecture.md](docs/architecture.md),
[docs/conversion_workflows.md](docs/conversion_workflows.md), and
[docs/api.md](docs/api.md). The reproducible Docker runtime and map regression
workflow are described in [docs/runtime.md](docs/runtime.md).

## License

AutoMapConverter is distributed under GPL-3.0-or-later. See
[LICENSE.txt](LICENSE.txt) and [NOTICE.md](NOTICE.md) for license notices.
