"""One invocation per task. Never import the engine in the HTTP server."""
import hashlib
import json
import os
import sys
import zipfile
from pathlib import Path

import requests
import yaml
from lxml import etree

from .models import TaskRequest


def write_json(path, value):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, ensure_ascii=False), encoding="utf-8")
    temporary.replace(path)


def validate_xml(path, source_format):
    parser = etree.XMLParser(resolve_entities=False, no_network=True, load_dtd=False)
    tree = etree.parse(str(path), parser)
    if tree.docinfo.doctype:
        raise ValueError("不允许包含 DTD 或外部实体的地图文件")
    root = tree.getroot()
    if source_format == "opendrive":
        if etree.QName(root).localname != "OpenDRIVE":
            raise ValueError("文件内容不是 OpenDRIVE")
    else:
        if root.tag != "osm":
            raise ValueError("文件内容不是 OSM XML")
        lanelets = root.xpath("./relation/tag[@k='type' and @v='lanelet']")
        if source_format == "lanelet2" and not lanelets:
            raise ValueError("未找到 Lanelet2 车道关系，请检查源格式")
        if source_format == "osm" and lanelets:
            raise ValueError("检测到 Lanelet2，请将源格式改为 Lanelet2")


def download(url, path, max_bytes):
    with requests.get(url, stream=True, timeout=(10, 60), allow_redirects=False) as response:
        if response.status_code != 200:
            raise RuntimeError("源文件下载失败，请重新提交任务")
        size = 0
        with path.open("wb") as stream:
            for chunk in response.iter_content(1024 * 1024):
                size += len(chunk)
                if size > max_bytes:
                    raise ValueError("地图文件超过大小限制")
                stream.write(chunk)


def clean_report(value, directory):
    # Reports may contain internal paths; downloads expose task-relative paths only.
    if isinstance(value, dict):
        return {k: clean_report(v, directory) for k, v in value.items()}
    if isinstance(value, list):
        return [clean_report(v, directory) for v in value]
    if isinstance(value, str):
        return value.replace(str(directory) + os.sep, "")
    return value


def execute(directory):
    request = TaskRequest.model_validate_json((directory / "request.json").read_text())
    stage = lambda name: write_json(directory / "stage.json", {"stage": name})
    stage("DOWNLOADING")
    source = directory / ("source.xodr" if request.sourceFormat == "opendrive" else "source.osm")
    target = directory / ("converted.xodr" if request.targetFormat == "opendrive" else "converted.osm")
    download(request.sourceUrl, source, request.maxBytes)
    stage("VALIDATING")
    validate_xml(source, request.sourceFormat)
    config = directory / "platform.yaml"
    config.write_text(yaml.safe_dump({
        "conversion": {"opendrive_version": request.opendriveVersion},
        "stage1": {"launch_viewer": False},
    }))
    stage("CONVERTING_AND_DIAGNOSING" if request.diagnose else "CONVERTING")
    from automap_converter import convert, convert_and_diagnose
    if request.diagnose:
        result = convert_and_diagnose(source, target, request.sourceFormat, request.targetFormat,
                                      diagnostics_dir=directory / "diagnostics", config_path=config)
    else:
        result = convert(source, target, request.sourceFormat, request.targetFormat, config_path=config)
    if not target.is_file() or not target.stat().st_size:
        raise RuntimeError("转换未生成有效输出文件")
    report = {"result": "NOT_CHECKED", "summary": {}, "conversion": result.conversion}
    if result.diagnostics_report:
        report = json.loads(result.diagnostics_report.with_suffix(".json").read_text())
    report = clean_report(report, directory)
    report_path = directory / "report.json"
    write_json(report_path, report)
    stage("UPLOADING")
    bundle = directory / "diagnostics.zip"
    with zipfile.ZipFile(bundle, "w", zipfile.ZIP_DEFLATED) as archive:
        archive.write(report_path, "report.json")
        # Include all stage reports, even when conversion used a composed route.
        for path in directory.rglob("*"):
            if path.is_file() and path.suffix in {".txt", ".json", ".xqar"} and path.name not in {
                "request.json", "result.json", "stage.json", "report.json"
            }:
                if path.suffix == ".json":
                    archive.writestr(str(path.relative_to(directory)), json.dumps(
                        clean_report(json.loads(path.read_text()), directory), ensure_ascii=False))
                elif path.suffix == ".txt":
                    archive.writestr(str(path.relative_to(directory)), clean_report(path.read_text(), directory))
                else:
                    archive.write(path, path.relative_to(directory))
    artifacts = []
    for kind, path in {"map": target, "report": report_path, "bundle": bundle}.items():
        if path.stat().st_size > 2 * 1024 * 1024 * 1024:
            raise ValueError("转换产物超过限制")
        with path.open("rb") as stream:
            response = requests.put(request.uploadUrls[kind], data=stream, timeout=(10, 120),
                                    allow_redirects=False)
        if response.status_code not in {200, 201, 204}:
            raise RuntimeError("结果上传失败，请重试任务")
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
        artifacts.append({"kind": kind, "size": path.stat().st_size, "sha256": digest.hexdigest()})
    quality = report.get("result", "NOT_CHECKED")
    if quality not in {"PASS", "PASS_WITH_WARNINGS", "FAIL", "ERROR", "NOT_CHECKED"}:
        quality = "NOT_CHECKED"
    write_json(directory / "result.json", {"quality": quality, "artifacts": artifacts,
                                            "toolVersion": "0.1.0"})


def main():
    directory = Path(sys.argv[1]).resolve()
    # Parent enforces wall time; prevent unbounded single-file/log growth on Linux.
    if os.name == "posix":
        import resource
        resource.setrlimit(resource.RLIMIT_FSIZE, (2 * 1024**3, 2 * 1024**3))
    try:
        execute(directory)
    except Exception as exc:
        # Requests exceptions can embed signed URLs: never serialize arbitrary exception text.
        error = str(exc) if isinstance(exc, ValueError) and not isinstance(exc, requests.RequestException) else "地图转换或结果传输失败，请检查输入地图或重试"
        write_json(directory / "result.json", {"error": error[:500]})
        print("Conversion failed:", type(exc).__name__)
        return 1
    finally:
        (directory / "request.json").unlink(missing_ok=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
