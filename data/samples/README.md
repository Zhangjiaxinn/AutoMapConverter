# Sample Maps

This directory stores input maps used by runnable examples and pull-request
regression. Add a map directly under its source-format directory and the
regression workflow will discover it automatically:

- `lanelet2/basic_intersection_area.osm`: Lanelet2 input for Lanelet2 -> OpenDRIVE and Lanelet2 -> OSM.
- `opendrive/commonroad_straight_road.xodr`: OpenDRIVE input for OpenDRIVE -> Lanelet2.
- `osm/ped_crossing.osm`: generic OSM input for OSM -> Lanelet2.

Each example writes its converted map and diagnostic reports under `data/results/`.

For every discovered map, CI runs each supported outgoing workflow: Lanelet2
maps run Lanelet2 -> OpenDRIVE and Lanelet2 -> OSM; OpenDRIVE maps run
OpenDRIVE -> Lanelet2 and OpenDRIVE -> OSM; OSM maps run OSM -> Lanelet2 and
OSM -> OpenDRIVE. Each case writes its target map and diagnostic reports to the
workflow artifact.
