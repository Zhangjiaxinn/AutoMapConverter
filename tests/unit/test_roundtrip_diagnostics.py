import pytest

from automap_converter.validation.diagnostics._roundtrip_contract import section_width
from automap_converter.validation.diagnostics.roundtrip_diagnostics import (
    Feature,
    FeatureSet,
    compare_feature_sets,
    write_summary,
)

FRAME = {"kind": "engineering", "id": "test-frame", "units": "m"}


def lane(key="a", *, y=0.0, width=3.5, ancestor=None):
    feature = Feature(
        key,
        [(0.0, y), (10.0, y)],
        width,
        "road",
        "yes",
        left=[(0.0, y + width / 2), (10.0, y + width / 2)],
        right=[(0.0, y - width / 2), (10.0, y - width / 2)],
    )
    if ancestor:
        feature.ancestors = [{"document": "source", "format": "lanelet2", "key": ancestor}]
    return feature


def maps(*targets, source=None):
    return (
        FeatureSet("lanelet2", source or [lane()], frame=FRAME, document="source"),
        FeatureSet("lanelet2", list(targets), frame=FRAME, document="target"),
    )


def metrics(result):
    return {m["name"]: m for m in result["metrics"]}


def test_unchanged_generated_lane_has_four_metrics():
    result = compare_feature_sets(*maps(lane("new", ancestor="a")))
    assert result["result"] == "PASS"
    assert len(result["metrics"]) == 4
    ms = metrics(result)
    assert ms["centerline_mean_error_m"]["pass_rate"] == 1
    assert ms["lane_width_mean_relative_error"]["total"] == 5
    assert ms["lane_width_mean_relative_error"]["pass_rate"] == 1
    assert ms["topology_edge_match_rate"]["pass_rate"] is None


def test_nearby_lane_without_lineage_is_not_a_match():
    result = compare_feature_sets(*maps(lane("new")))
    assert metrics(result)["centerline_mean_error_m"]["pass_rate"] == 0
    assert result["evidence"]["unmapped_source_keys"] == ["a"]


def test_lineage_from_another_document_is_not_a_match():
    generated = lane("new", ancestor="a")
    generated.ancestors[0]["document"] = "another-source"
    assert (
        metrics(compare_feature_sets(*maps(generated)))["centerline_mean_error_m"]["pass_rate"] == 0
    )


def test_shift_is_measured_instead_of_aligned_away():
    result = compare_feature_sets(*maps(lane("new", y=1.5, ancestor="a")))
    metric = metrics(result)["centerline_mean_error_m"]
    assert metric["value"] == pytest.approx(1.5)
    assert metric["pass_rate"] == 0
    assert metric["status"] == "FAIL"


def test_extension_is_detected_by_paired_points():
    generated = lane("longer", ancestor="a")
    generated.points = [(0.0, 0.0), (20.0, 0.0)]
    result = compare_feature_sets(*maps(generated))
    assert metrics(result)["centerline_mean_error_m"]["value"] == pytest.approx(5.0)


def test_each_lane_mean_must_pass_not_just_global_mean():
    first, second = lane("a"), lane("b", y=10)
    actual_a = lane("x", ancestor="a")
    actual_b = lane("y", y=11.5, ancestor="b")
    result = compare_feature_sets(*maps(actual_a, actual_b, source=[first, second]))
    metric = metrics(result)["centerline_mean_error_m"]
    assert metric["value"] == pytest.approx(0.75)
    assert metric["status"] == "FAIL"
    assert metric["pass_rate"] == 0.5
    assert (metric["passed"], metric["total"]) == (1, 2)
    assert metric["pass_rate_basis"] == "source_line_mean"


def test_coordinate_display_counts_line_means_not_individual_points():
    generated = lane("new", ancestor="a")
    generated.points = [(0, 0), (10, 1.5)]
    result = compare_feature_sets(*maps(generated))
    metric = metrics(result)["centerline_mean_error_m"]
    evidence = result["evidence"]["lanes"][0]
    assert max(evidence["point_errors_m"]) > 1
    assert evidence["point_passed"] < evidence["point_total"]
    assert metric["value"] == pytest.approx(0.75)
    assert metric["status"] == "PASS"
    assert metric["pass_rate"] == 1
    assert (metric["passed"], metric["total"]) == (1, 1)


def test_unmatched_line_remains_in_mean_pass_rate_denominator():
    result = compare_feature_sets(*maps(lane("new", ancestor="a"), source=[lane(), lane("b")]))
    metric = metrics(result)["centerline_mean_error_m"]
    assert (metric["passed"], metric["total"]) == (1, 2)
    assert metric["pass_rate"] == 0.5
    assert metric["status"] == "FAIL"


def test_summary_uses_line_mean_label_and_percentage(tmp_path):
    result = compare_feature_sets(*maps(lane("new", ancestor="a"), source=[lane(), lane("b")]))
    write_summary(tmp_path, [{**result, "case": "example", "source": "example.osm", "pair": "test"}])
    markdown = (tmp_path / "summary.md").read_text()
    assert "Line means <=1m" in markdown
    assert "50.00%" in markdown
    assert "counts individual points" not in markdown


def test_width_checks_local_sections_not_lane_average():
    generated = lane("new", ancestor="a")
    generated.left = [(0, 1.75), (0.5, 1.75), (5, 2.75), (9.5, 1.75), (10, 1.75)]
    assert section_width(generated, 0.5) == pytest.approx(4.5)
    metric = metrics(compare_feature_sets(*maps(generated)))["lane_width_mean_relative_error"]
    assert metric["total"] == 5
    assert metric["pass_rate"] == 0
    assert metric["status"] == "FAIL"


def test_invalid_target_width_cannot_be_skipped():
    generated = lane("new", ancestor="a")
    generated.left, generated.right, generated.width = [], [], None
    metric = metrics(compare_feature_sets(*maps(generated)))["lane_width_mean_relative_error"]
    assert metric["pass_rate"] == 0
    assert metric["status"] == "FAIL"


def test_unknown_source_width_is_not_invented():
    source = lane()
    source.left, source.right, source.width = [], [], None
    metric = metrics(compare_feature_sets(*maps(lane("new", ancestor="a"), source=[source])))[
        "lane_width_mean_relative_error"
    ]
    assert metric["status"] == "SKIP"
    assert metric["total"] == 0


def test_attribute_fraction_counts_complete_lanes():
    generated = lane("new", ancestor="a")
    generated.direction = "no"
    metric = metrics(compare_feature_sets(*maps(generated)))["type_direction_attribute_match_rate"]
    assert metric["pass_rate"] == 0
    assert metric["total"] == 1


def test_rules_must_remain_on_the_correct_lane_and_position():
    source = lane()
    source.rules = [{"kind": "traffic_light", "value": "", "points": [(5, 0)]}]
    generated = lane("new", ancestor="a")
    generated.rules = [{"kind": "traffic_light", "value": "", "points": [(8, 0)]}]
    result = compare_feature_sets(*maps(generated, source=[source]))
    assert metrics(result)["type_direction_attribute_match_rate"]["pass_rate"] == 0


def test_exact_topology_and_lateral_relations_are_compared():
    first, second = lane("a"), lane("b", y=3.5)
    first.relations = {"left": ["b"]}
    second.relations = {"right": ["a"]}
    a, b = lane("x", ancestor="a"), lane("y", y=3.5, ancestor="b")
    a.relations = {"left": ["y"]}
    b.relations = {"right": ["x"]}
    a.measurements["native_boundary_ids"] = {"left": "shared", "right": "lower"}
    b.measurements["native_boundary_ids"] = {"left": "upper", "right": "shared"}
    assert (
        metrics(compare_feature_sets(*maps(a, b, source=[first, second])))[
            "topology_edge_match_rate"
        ]["pass_rate"]
        == 1
    )
    b.relations = {}
    assert (
        metrics(compare_feature_sets(*maps(a, b, source=[first, second])))[
            "topology_edge_match_rate"
        ]["pass_rate"]
        == 0.5
    )


def test_incompatible_engineering_frames_do_not_compare_raw_numbers():
    source, target = maps(lane("new", ancestor="a"))
    target.frame = {"kind": "engineering", "id": "another-frame"}
    result = compare_feature_sets(source, target)
    assert result["evidence"]["coordinate_error"]
    assert metrics(result)["centerline_mean_error_m"]["status"] == "FAIL"
