from automap_converter.validation.diagnostics.roundtrip_diagnostics import (
    Feature,
    FeatureSet,
    compare_feature_sets,
)


def test_roundtrip_matching_allows_source_segments_to_merge() -> None:
    source = FeatureSet(
        "lanelet2",
        [
            Feature("a", [(0.0, 0.0), (5.0, 0.0)], 3.5, "road", "yes"),
            Feature("b", [(5.0, 0.0), (10.0, 0.0)], 3.5, "road", "yes"),
        ],
        topology_edges={("a", "b")},
    )
    target = FeatureSet(
        "lanelet2",
        [Feature("merged", [(0.0, 0.0), (10.0, 0.0)], 3.5, "road", "yes")],
    )

    result = compare_feature_sets(source, target)

    assert result["result"] == "PASS"
    metrics = {metric["name"]: metric for metric in result["metrics"]}
    assert metrics["feature_coverage"]["value"] == 1.0
    assert metrics["centerline_mean_error_m"]["value"] == 0.0
    assert metrics["topology_edge_match_rate"]["value"] == 1.0


def test_roundtrip_matching_detects_reversed_direction() -> None:
    source = FeatureSet(
        "lanelet2",
        [Feature("a", [(2.0, 0.0), (8.0, 0.0)], 3.5, "road", "yes")],
    )
    target = FeatureSet(
        "lanelet2",
        [Feature("reversed", [(10.0, 0.0), (0.0, 0.0)], 3.5, "road", "yes")],
    )

    result = compare_feature_sets(source, target)
    metrics = {metric["name"]: metric for metric in result["metrics"]}

    assert metrics["feature_coverage"]["value"] == 1.0
    assert metrics["type_direction_attribute_match_rate"]["value"] == 0.5
