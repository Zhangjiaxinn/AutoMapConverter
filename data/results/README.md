# Conversion Results

Runnable examples and batch workflows write converted maps and diagnostics here.
Generated files are ignored by default so local runs do not alter the repository
state.

```text
<conversion>/<source-map-stem>/run_001/
  <target-map>
  <source-map-stem>_occupancy.tif       # Lanelet2 -> Raster only
  <source-map-stem>_preview.png         # Lanelet2 -> Raster only
  <source-map-stem>_occupancy_preview.png
  <source-map-stem>_raster_metadata.json
  <source-map-stem>_prepared.<suffix>  # only if source repair changed a copy
  <target-map>_diagnostics.json
  manifest.json

<conversion>/_batches/batch_001/
  summary.txt
  summary.json
```

Every rerun receives the next `run_###` directory, preserving earlier maps and
reports for comparison.

Supported conversion directories are `lanelet2_to_opendrive`,
`lanelet2_to_osm`, `opendrive_to_lanelet2`, `opendrive_to_osm`,
`osm_to_lanelet2`, `osm_to_opendrive`, and `lanelet2_to_raster`. They are
created when that direction is first run; empty directories are intentionally
not stored in Git.

Normal conversion does not create `preprocessed_sources/` or `target_repairs/`
directories. A source repair is written directly into its `run_###` directory
only when it changes the source; otherwise no prepared copy is created. The
explicit `automap-converter prepare-source` command still writes only to the
path supplied by the user.
