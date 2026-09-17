# Command Line

Convert and diagnose one map:

```bash
automap-converter convert --from lanelet2 --to opendrive \
  --input source.osm --output target.xodr --diagnose
```

Convert a directory using the same validation workflow:

```bash
automap-converter batch --from osm --to lanelet2 \
  --input-dir path/to/osm_maps --output-dir data/results
```

Generate semantic and occupancy raster products:

```bash
automap-converter convert --from lanelet2 --to raster \
  --input source.osm --output semantic_map.tif --diagnose
```

The primary `--output` is the semantic GeoTIFF. Occupancy, preview, metadata
and diagnostic companions use the same stem in the same directory.

For batch commands, `--output-dir` is the shared results root. The command
creates each map's next `run_###` directory and writes the aggregate summary to
`<output-dir>/<conversion>/_batches/batch_###/`.

During conversion, a known source compatibility repair is applied only if it
changes a run-local `<map>_prepared.*` copy. The original source is never
rewritten. Create a separate prepared source copy at a location of your own
choosing with:

```bash
automap-converter prepare-source --from opendrive \
  --input source.xodr --output prepared_source.xodr
```

`convert` and `batch` never postprocess or repair the generated target map.
The explicit `prepare-source` command remains useful when the prepared copy
should be reused or reviewed before conversion.

The GUI package is currently a reserved integration layer. When added, it will
call this API and command workflow rather than reimplement conversion logic.
