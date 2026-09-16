# Development

## Continuous Integration

The CI has two required jobs on every push and pull request:

- `unit`: package installation, source compilation, unit tests and CLI smoke check;
- `map-regression`: the Docker runtime executes seven conversion workflows over
  the curated small maps in `data/samples/`, writes diagnostics, and uploads
  the generated maps and reports as a workflow artifact.

The container fixes Python 3.10, Lanelet2 1.2.2, esmini and the ASAM OpenDRIVE
Quality Checker. It also includes `osmium-tool` for OSM reference checks.
`odrviewer` is installed for local manual inspection but is deliberately not
opened in headless CI.

Build and enter the same runtime locally:

```bash
docker build -t automap-converter:ci .
docker run --rm -it -v "$PWD:/workspace" automap-converter:ci bash
python scripts/run_regression.py --output-dir data/results/regression
```

Every normal code change first passes the small curated regression set. Medium
collections and full-scale pressure maps are intended for manually dispatched
or scheduled workflows once those curated datasets are added to the repository.

```bash
python -m compileall -q src/automap_converter
pytest tests/unit
automap-converter --help
```

Add a unit test for every local behavior change and a compact regression map
for each general conversion fix. Keep generated artifacts, large external data
and credentials out of commits.
