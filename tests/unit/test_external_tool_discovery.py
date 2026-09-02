from pathlib import Path

from automap_converter.validation.diagnostics.lanelet2_opendrive_diagnostics import (
    Lanelet2OpenDriveDiagnostics,
)


def test_asam_checker_uses_explicit_environment_command(monkeypatch, tmp_path: Path) -> None:
    checker = tmp_path / "qc_opendrive"
    checker.write_text("#!/bin/sh\nexit 0\n", encoding="utf-8")
    checker.chmod(0o755)
    monkeypatch.setenv("AUTOMAP_CONVERTER_ASAM_QC_CMD", str(checker))

    assert Lanelet2OpenDriveDiagnostics._find_asam_quality_checker() == str(checker)
