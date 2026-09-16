import json
from pathlib import Path

import rasterio

from automap_converter import convert_and_diagnose
from automap_converter.conversion.lanelet2_to_raster.converter import SEMANTIC_BANDS

LANELET2_FIXTURE = """\
<?xml version="1.0" encoding="UTF-8"?>
<osm version="0.6" generator="AutoMapConverter-test">
  <node id="1" lat="" lon=""><tag k="local_x" v="0"/><tag k="local_y" v="0"/></node>
  <node id="2" lat="0" lon="0"><tag k="local_x" v="10"/><tag k="local_y" v="0"/></node>
  <node id="3" lat="0" lon="0"><tag k="local_x" v="0"/><tag k="local_y" v="3"/></node>
  <node id="4" lat="0" lon="0"><tag k="local_x" v="10"/><tag k="local_y" v="3"/></node>
  <node id="5" lat="0" lon="0"><tag k="local_x" v="0"/><tag k="local_y" v="6"/></node>
  <node id="6" lat="0" lon="0"><tag k="local_x" v="10"/><tag k="local_y" v="6"/></node>
  <way id="10"><nd ref="1"/><nd ref="2"/></way>
  <way id="11"><nd ref="3"/><nd ref="4"/></way>
  <way id="12"><nd ref="5"/><nd ref="6"/></way>
  <relation id="100">
    <member type="way" ref="10" role="left"/>
    <member type="way" ref="11" role="right"/>
    <tag k="type" v="lanelet"/><tag k="subtype" v="road"/>
  </relation>
  <relation id="101">
    <member type="way" ref="11" role="left"/>
    <member type="way" ref="12" role="right"/>
    <tag k="type" v="lanelet"/><tag k="subtype" v="road"/>
  </relation>
  <relation id="102" action="delete">
    <tag k="type" v="lanelet"/>
  </relation>
</osm>
"""


def test_lanelet2_raster_products_and_diagnostics(tmp_path: Path) -> None:
    source = tmp_path / "two_lanes.osm"
    target = tmp_path / "two_lanes.tif"
    source.write_text(LANELET2_FIXTURE, encoding="utf-8")

    result = convert_and_diagnose(
        source,
        target,
        source_format="lanelet2",
        target_format="raster",
        diagnostics_dir=tmp_path,
    )

    assert set(result.artifacts) == {
        "semantic",
        "occupancy",
        "semantic_preview",
        "occupancy_preview",
        "metadata",
    }
    assert all(path.is_file() for path in result.artifacts.values())
    with rasterio.open(target) as dataset:
        assert dataset.count == 6
        assert dataset.descriptions == SEMANTIC_BANDS
        assert all(dtype == "uint16" for dtype in dataset.dtypes)
    metadata = json.loads(result.artifacts["metadata"].read_text(encoding="utf-8"))
    assert metadata["source_lanelets"] == 2
    assert metadata["rasterized_lanelets"] == 2
    report = json.loads(result.diagnostics_report.read_text(encoding="utf-8"))
    assert report["result"] == "PASS"
    assert report["metrics"]["relation_coverage"] == 1.0
