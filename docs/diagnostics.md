# Diagnostics

Stage 1 checks source legality, target parse/loadability and optional viewer
availability. Stage 2 checks target conformance, geometry, topology, references
and source-element mapping at the smallest traceable road, lanelet, lane or
regulatory element.

The four atomic workflow profiles are implemented independently:

- `lanelet2_opendrive_diagnostics.py`
- `opendrive_lanelet2_diagnostics.py`
- `osm_lanelet2_diagnostics.py`
- `lanelet2_osm_diagnostics.py`

Shared report serialization and XML statistic collection are private runtime
components. Each public profile fixes the source/target formats and selects its
own geometry, topology and semantic checks.

Round-trip diagnostics compare A -> B -> A' without treating source IDs or
provenance as proof that a conversion succeeded. Reports are emitted as
human-readable `.txt` and structured `.json` files.

The two composed workflows, OSM -> OpenDRIVE and OpenDRIVE -> OSM, retain the
reports for both atomic stages plus one concise composed report index.
