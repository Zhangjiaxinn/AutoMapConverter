# Conversion Rules

- Direction modules use `snake_case`; classes use `PascalCase`; constants use
  `UPPER_CASE`.
- A general conversion fix belongs in the relevant workflow directory.
- A map-specific exception must be explicit, named, switchable and covered by a
  regression case before it is retained.
- Intermediate representations are used only where a workflow requires them;
  do not add duplicate direct converters merely to expose another route.
- Do not use provenance tags to recreate information that was never converted.
  Diagnostics may use provenance for tracing, but round-trip quality is based
  on geometry, topology and semantic comparison.
- Raster products keep non-spatial graph relationships and source identities
  in a versioned metadata sidecar; pixel values alone are not treated as a
  lossless vector representation.
