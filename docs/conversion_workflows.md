# Conversion Workflows

## Lanelet2 -> OpenDRIVE

Parse Lanelet2, enrich topology when the optional routing plugin is available,
then build OpenDRIVE road, lane, junction, signal and object elements directly.

## OpenDRIVE -> Lanelet2

Parse OpenDRIVE, construct an intermediate lane network, then write Lanelet2.
Road/lane provenance and mapping diagnostics remain available for inspection.

## OSM -> Lanelet2

Build the OSM road graph, derive lanelets and traffic controls, then
write Lanelet2. Pedestrian/cyclist sublayers are configurable.

## Lanelet2 -> OSM

Generate generic OSM centerline roads and selected road-level rules. This route
is intentionally lossy because OSM does not natively encode every Lanelet2
lane-level relation, area or topology detail.

There is no duplicate direct OSM <-> OpenDRIVE module. Compose the two
directions through Lanelet2 when that transformation is required.

## OSM -> OpenDRIVE

Compose OSM -> CommonRoad -> Lanelet2, then build OpenDRIVE directly from the
resulting Lanelet2 map. The output directory retains the intermediate Lanelet2
map and one diagnostic report per stage, plus a concise composed-report index.

## OpenDRIVE -> OSM

Compose OpenDRIVE -> CommonRoad -> Lanelet2, then export generic OSM from the
resulting Lanelet2 map. This route inherits the intentional Lanelet2 -> OSM
information loss and reports it explicitly rather than recreating source data.
