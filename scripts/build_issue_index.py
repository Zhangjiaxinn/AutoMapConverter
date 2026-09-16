"""Build a compact issue index from the latest available diagnostic reports."""

from __future__ import annotations

import json
from collections import Counter, defaultdict
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RESULTS = ROOT / "data" / "results"
OUTPUT = RESULTS / "conversion_issue_index.md"
ROUTES = (
    "lanelet2_to_opendrive",
    "lanelet2_to_osm",
    "lanelet2_to_raster",
    "opendrive_to_lanelet2",
    "opendrive_to_osm",
    "osm_to_lanelet2",
    "osm_to_opendrive",
)
ROUTE_SOURCE_FORMAT = {
    "lanelet2_to_opendrive": "lanelet2",
    "lanelet2_to_osm": "lanelet2",
    "lanelet2_to_raster": "lanelet2",
    "opendrive_to_lanelet2": "opendrive",
    "opendrive_to_osm": "opendrive",
    "osm_to_lanelet2": "osm",
    "osm_to_opendrive": "osm",
}
CURRENT_ROUNDTRIP_TIMESTAMP = datetime.fromisoformat("2026-09-16T00:00:00").timestamp()


def _load(path: Path) -> dict:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return {}


def _link(path: Path) -> str:
    return f"[报告](./{path.relative_to(RESULTS).as_posix()})"


def _latest_single_reports() -> list[dict]:
    rows = []
    for route in ROUTES:
        route_dir = RESULTS / route
        if not route_dir.is_dir():
            continue
        source_dir = ROOT / "data" / "samples" / ROUTE_SOURCE_FORMAT[route]
        source_names = {path.stem for path in source_dir.iterdir() if path.is_file()}
        for map_dir in sorted(route_dir.iterdir()):
            if not map_dir.is_dir() or map_dir.name not in source_names:
                continue
            reports = [
                path
                for path in map_dir.glob("run_*/*_diagnostics.json")
                if path.parent.name.startswith("run_")
            ]
            if not reports:
                continue
            path = max(reports, key=lambda item: item.stat().st_mtime)
            payload = _load(path)
            rows.append(
                {
                    "route": route,
                    "map": map_dir.name,
                    "path": path,
                    "payload": payload,
                    "result": str(payload.get("result", "UNKNOWN")),
                }
            )
    return rows


def _failed_categories(payload: dict) -> set[str]:
    categories = {
        str(item.get("category", "acceptance"))
        for item in payload.get("acceptance", [])
        if isinstance(item, dict) and item.get("status") in {"FAIL", "ERROR"}
    }
    diagnostics = payload.get("diagnostics", {})
    if isinstance(diagnostics, dict):
        for group in diagnostics.values():
            if not isinstance(group, list):
                continue
            categories.update(
                str(item.get("category", "diagnostic"))
                for item in group
                if isinstance(item, dict) and item.get("status") in {"FAIL", "ERROR"}
            )
    return categories or {"conversion-or-pipeline"}


def _latest_roundtrips() -> list[dict]:
    latest: dict[tuple[str, str], dict] = {}
    for path in RESULTS.glob("roundtrip/**/roundtrip_diagnostics.json"):
        payload = _load(path)
        case = str(payload.get("case", "unknown"))
        source = Path(str(payload.get("source", path.parent.name))).stem
        source = source.removesuffix("_prepared")
        key = case, source
        row = {"case": case, "map": source, "path": path, "payload": payload}
        if key not in latest or path.stat().st_mtime > latest[key]["path"].stat().st_mtime:
            latest[key] = row
    return sorted(latest.values(), key=lambda row: (row["case"], row["map"]))


def main() -> int:
    singles = _latest_single_reports()
    roundtrips = _latest_roundtrips()
    lines = [
        "# Conversion Issue Index",
        "",
        f"生成时间：{datetime.now().isoformat(timespec='seconds')}",
        "",
        "> 本索引从每张地图的最新报告自动生成。已在最新报告中消失的问题不会继续列出。",
        "> 双向评价器于 2026-09-16 修正了分段合并误判；此前报告列为待复测，不计入当前确认失败。",
        "",
        "## 1. 单向转换汇总",
        "",
        "| 链路 | 报告数 | PASS | PASS_WITH_WARNINGS | FAIL/ERROR |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for route in ROUTES:
        route_rows = [row for row in singles if row["route"] == route]
        counts = Counter(row["result"] for row in route_rows)
        lines.append(
            f"| `{route}` | {len(route_rows)} | {counts['PASS']} | "
            f"{counts['PASS_WITH_WARNINGS']} | {counts['FAIL'] + counts['ERROR']} |"
        )

    failures: dict[str, list[dict]] = defaultdict(list)
    for row in singles:
        if row["result"] not in {"FAIL", "ERROR"}:
            continue
        for category in _failed_categories(row["payload"]):
            failures[category].append(row)
    lines.extend(["", "## 2. 单向失败按根因归类", ""])
    if not failures:
        lines.append("当前最新单向报告中没有 FAIL/ERROR。")
    for category, rows in sorted(failures.items(), key=lambda item: (-len(item[1]), item[0])):
        lines.extend([f"### `{category}`：{len(rows)} 条", ""])
        for row in rows:
            lines.append(
                f"- `{row['map']}` | `{row['route']}` | {_link(row['path'])}"
            )
        lines.append("")

    verified = [
        row
        for row in roundtrips
        if row["path"].stat().st_mtime >= CURRENT_ROUNDTRIP_TIMESTAMP
    ]
    pending = [row for row in roundtrips if row not in verified]
    lines.extend(
        [
            "## 3. 双向转换",
            "",
            f"新评价器已复核 {len(verified)} 条；旧评价器待复测 {len(pending)} 条。",
            "",
        ]
    )
    for row in verified:
        failed_metrics = [
            str(metric.get("name"))
            for metric in row["payload"].get("metrics", [])
            if isinstance(metric, dict) and metric.get("status") == "FAIL"
        ]
        lines.append(
            f"- `{row['map']}` | `{row['case']}` | "
            f"{row['payload'].get('result', 'UNKNOWN')} | "
            f"{', '.join(failed_metrics) or '-'} | {_link(row['path'])}"
        )
    pending_by_case = Counter(row["case"] for row in pending)
    if pending_by_case:
        lines.extend(["", "### 待复测基线", ""])
        for case, count in sorted(pending_by_case.items()):
            lines.append(f"- `{case}`：{count} 条")

    warning_counts = Counter()
    for row in singles:
        payload = row["payload"]
        for item in payload.get("acceptance", []):
            if isinstance(item, dict) and item.get("status") in {"WARN", "SKIP"}:
                warning_counts[str(item.get("category", "acceptance"))] += 1
        diagnostics = payload.get("diagnostics", {})
        if isinstance(diagnostics, dict):
            for group in diagnostics.values():
                if isinstance(group, list):
                    for item in group:
                        if isinstance(item, dict) and item.get("status") in {"WARN", "SKIP"}:
                            warning_counts[str(item.get("category", "diagnostic"))] += 1
    lines.extend(["", "## 4. 警告与跳过项", ""])
    for category, count in warning_counts.most_common():
        lines.append(f"- `{category}`：{count} 条")
    lines.extend(
        [
            "",
            "`PASS_WITH_WARNINGS` 表示结果可生成但存在已知信息损失；`SKIP` 通常表示可选外部工具未安装，不等同于转换失败。",
            "",
        ]
    )
    OUTPUT.write_text("\n".join(lines), encoding="utf-8")
    print(OUTPUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
