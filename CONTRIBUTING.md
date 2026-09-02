# Contributing

## Development Setup

```bash
python -m venv .venv
source .venv/bin/activate
pip install -e ".[dev]"
```

## Rules

- Keep format adapters in `src/automap_converter/formats/` and conversion behavior in the matching workflow directory.
- Add application-facing behavior through `src/automap_converter/api/`.
- Do not commit generated artifacts, large maps, virtual environments or credentials.
- Add a focused unit test or compact regression map for each general conversion fix.
- Record user-visible behavior changes in `CHANGELOG.md`.

## Validation

```bash
python -m compileall -q src/automap_converter
pytest
automap-converter convert --help
```
