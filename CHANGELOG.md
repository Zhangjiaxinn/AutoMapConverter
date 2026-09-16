# Changelog

## 0.1.0 - Unreleased

- Established the public Python API and command-line interface for six vector-map workflows.
- Added a seventh Lanelet2 -> Raster route with semantic/occupancy GeoTIFFs,
  preview images, topology metadata and product diagnostics.
- Added source prechecks, two-stage diagnostics and vector-map round-trip evaluation.
- Added Lanelet2/OpenDRIVE mapping support for road geometry, topology, traffic controls and map objects.
- Organized the implementation around self-contained conversion workflows and format-specific parsers.
- Corrected OpenDRIVE junction contact-section handling and separated XML laneLink
  integrity from driving-lane topology evaluation.
- Prevented empty Lanelet2 right-of-way relations and ordered OSM elements for
  `osmium check-refs` compatibility.
- Made round-trip geometry evaluation aware of legitimate segment merges and
  added spatial indexing for large maps.
- Added a documented nuScenes-based showcase set for Lanelet2, OpenDRIVE, OSM
  and semantic raster output.
