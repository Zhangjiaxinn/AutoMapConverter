# Reproducible Runtime and Regression

`Dockerfile` defines the Linux runtime used by map regression CI. It fixes:

- Python 3.10 and Lanelet2 1.2.2;
- system geometry and OpenGL runtime libraries;
- esmini 3.6.0, including `odrviewer` and `odrplot`;
- ASAM OpenDRIVE Quality Checker 1.0.0 in an isolated virtual environment;
- `osmium-tool` for OSM reference validation.

Build and use the image locally:

```bash
docker build -t automap-converter:ci .
docker run --rm -it -v "$PWD:/workspace" automap-converter:ci bash
python scripts/run_regression.py --output-dir data/results/regression
```

The sample-map regression automatically discovers every supported input under
`data/samples/lanelet2`, `data/samples/opendrive`, and `data/samples/osm`. It
runs every applicable outgoing workflow and writes converted maps and
diagnostics below the selected output directory. A result is accepted when
there is no hard diagnostic failure; non-fatal warnings remain visible in the
uploaded reports.

On GitHub, the `map-regression` job builds this image on every push and pull
request, runs the complete current sample directory, and uploads generated maps
and reports as the `sample-map-regression` artifact. Adding a map to one of the
three sample directories is therefore sufficient to include it in every future
pull-request check.

`odrviewer` is installed in the image for local use. CI has no interactive
desktop, so it validates OpenDRIVE by running `odrplot`; manual visual
comparison remains a local desktop step.
