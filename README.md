# AutoMapConverter

AutoMapConverter is a multi-format road-map conversion and quality-analysis
toolkit for autonomous-driving research, simulation and data engineering. It
provides inspectable workflows across Lanelet2, OpenDRIVE and OpenStreetMap
(OSM), plus topology-aware Lanelet2 rasterization for planning, learning and
occupancy-grid applications.

The system coordinates geometry reconstruction, topology mapping, semantic
preservation and acceptance analysis across every conversion workflow. Each
public route has a stable Python/CLI entry point, structured diagnostics and
reproducible regression execution.

![AutoMapConverter system architecture](docs/assets/system_architecture.png)

## Development Status

AutoMapConverter `0.1.0` is the initial public release, supporting seven
map-conversion workflows through unified Python/CLI interfaces and structured
diagnostics.

### Repository Timeline

- [x] **2026-09:** Released AutoMapConverter `0.1.0` with six vector conversion
  routes, Lanelet2 semantic rasterization, unified Python/CLI interfaces,
  structured diagnostics, Docker runtime and regression CI.

## Engineering Scope

- **Multi-format ingestion:** format-aware parsing, coordinate normalization
  and source prechecks before conversion rules execute.
- **High-fidelity reconstruction:** explicit treatment of road geometry,
  lane-level topology, junction structure and traffic-control semantics.
- **Traceable quality evidence:** machine-readable acceptance, geometry,
  topology, element-mapping and round-trip metrics with source-to-result
  traceability.
- **Operational integration:** one Python API, one CLI contract, numbered
  result runs, configurable execution and Docker-backed regression CI.

## Supported Workflows

| Source | Target | Internal workflow |
| --- | --- | --- |
| Lanelet2 | OpenDRIVE | direct |
| OpenDRIVE | Lanelet2 | OpenDRIVE -> intermediate lane-network model -> Lanelet2 |
| OSM | Lanelet2 | OSM -> intermediate road-graph model -> Lanelet2 |
| Lanelet2 | OSM | direct, intentionally lossy |
| OSM | OpenDRIVE | OSM -> Lanelet2 -> OpenDRIVE |
| OpenDRIVE | OSM | OpenDRIVE -> Lanelet2 -> OSM |
| Lanelet2 | Raster | direct relation-aware semantic and occupancy rasterization |

The two composed routes reuse the explicit Lanelet2 workflows as their shared
conversion foundation. Raster is intentionally a one-way product format: the
vector source remains the authoritative map.

## Installation

```bash
git clone https://github.com/Zhangjiaxinn/AutoMapConverter.git
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
Python 3.10-3.11 environments. RViz and JOSM remain optional desktop
applications for manual vector-map comparison.

## Quick Start

```bash
automap-converter convert --from lanelet2 --to opendrive \
  --input input_map.osm --output output_map.xodr --diagnose
```

Normal conversion never overwrites the source map or applies post-conversion
repairs to the target map. It may apply a documented, conservative source
compatibility repair to a run-local copy when a repair rule actually changes
the source. That `<map>_prepared.osm` or `<map>_prepared.xodr` copy is stored
beside the target and recorded in the diagnostics and `manifest.json`.

To create a prepared source copy at a location of your own choosing, use:

```bash
automap-converter prepare-source --from opendrive \
  --input source.xodr --output prepared_source.xodr
```

The command records every applied normalization and never overwrites the
original source. Target-map problems remain visible in diagnostics and should
be corrected in the converter or inspected manually; they are never hidden by
an automatic target repair.

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

- **Single-direction diagnostics:** source validation, conditional run-local
  source preparation, target parsing/loading, geometry, topology, references,
  source-element mapping and raster-product consistency.
- **Round-trip diagnostics:** optional A -> B -> A' comparison for geometry,
  topology and semantics.

Each conversion run writes one concise, structured `.json` diagnostic report.
When configured, the raw ASAM OpenDRIVE Quality Checker `.xqar` result is
retained beside it.

## Result Layout

Runnable examples and batch commands store results by conversion direction,
source-map name and numbered attempt. Repeating a conversion never overwrites a
previous result:

```text
data/results/
  <source>_to_<target>/
    <map_name>/
      run_001/
        <converted_map>
        <map_name>_diagnostics.json
        manifest.json
        <workflow-specific companion products>
      run_002/
    _batches/
      <batch_name>/
        summary.txt
        summary.json
```

The seven conversion-direction directories are created on first use. This keeps
the repository free of empty result folders while retaining a stable layout for
all generated maps and reports.

All supported directions use this outer layout. Workflow-specific companion
products and composed-route evidence remain in the same run directory. A
prepared source appears only when a compatibility rule changes the input copy.
`manifest.json` records the source, target, diagnostic report, result and
elapsed time for one attempt.

## Layout

```text
src/automap_converter/
  formats/                 # Lanelet2/OpenDRIVE/OSM adapters and models
  conversion/              # six reusable conversion modules
    lanelet2_to_opendrive/
    lanelet2_to_osm/
    lanelet2_to_raster/
    opendrive_to_commonroad/
    osm_to_commonroad/
    commonroad_to_lanelet2/
  validation/              # source checks, single-direction and round-trip diagnostics
  api/                     # Python workflow API
  visualization/           # viewer integrations
  gui/                     # reserved GUI shell, calls API only
```

`data/samples/` contains compact runnable inputs. `data/results/` is the
standard destination for converted maps and their diagnostic reports.

Public YAML profiles in `configs/` control Lanelet2 topology enrichment, OSM
sublayer extraction, the OpenDRIVE output version, raster production, viewer
launch behavior and diagnostic groups. Use
`--config configs/default.yaml` to select a profile explicitly.

See [docs/architecture.md](docs/architecture.md),
[docs/conversion_workflows.md](docs/conversion_workflows.md), and
[docs/lanelet2_raster.md](docs/lanelet2_raster.md). The reproducible Docker
runtime and map regression workflow are described in
[docs/runtime.md](docs/runtime.md).

## License

AutoMapConverter is distributed under GPL-3.0-or-later. See
[LICENSE.txt](LICENSE.txt) and [NOTICE.md](NOTICE.md) for license notices.
