# Command Line

Convert and diagnose one map:

```bash
automap-converter convert --from lanelet2 --to opendrive \
  --input source.osm --output target.xodr --diagnose
```

Convert a directory using the same validation workflow:

```bash
automap-converter batch --from osm --to lanelet2 \
  --input-dir path/to/osm_maps --output-dir path/to/results
```

The GUI package is currently a reserved integration layer. When added, it will
call this API and command workflow rather than reimplement conversion logic.
