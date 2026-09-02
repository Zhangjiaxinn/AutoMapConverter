# Examples

Each conversion direction has a standalone runnable example. The scripts use
compact inputs from `data/samples/`, write converted maps into `data/results/`,
and request Stage 1 and Stage 2 diagnostics in the same output directory.

| Script | Input | Output |
| --- | --- | --- |
| `examples/lanelet2_to_opendrive.py` | Lanelet2 | OpenDRIVE |
| `examples/opendrive_to_lanelet2.py` | OpenDRIVE | Lanelet2 |
| `examples/osm_to_lanelet2.py` | OSM | Lanelet2 |
| `examples/lanelet2_to_osm.py` | Lanelet2 | OSM |
| `examples/osm_to_opendrive.py` | OSM | OpenDRIVE |
| `examples/opendrive_to_osm.py` | OpenDRIVE | OSM |
| `examples/osm_to_opendrive.py` | OSM | OpenDRIVE |
| `examples/opendrive_to_osm.py` | OpenDRIVE | OSM |

Each script also contains a commented batch-conversion invocation. Uncomment it
and replace the input directory when processing a map collection.
