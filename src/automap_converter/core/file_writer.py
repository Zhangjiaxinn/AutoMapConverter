"""Minimal CommonRoad writer used by intermediate conversion workflows."""

from commonroad.common.file_writer import CommonRoadFileWriter
from commonroad.common.writer.file_writer_interface import OverwriteExistingFile


class CommonRoadScenarioWriter(CommonRoadFileWriter):
    """Project-named alias for the standard CommonRoad scenario writer.

    Intermediate CommonRoad files are written without hidden repair. Validation
    remains explicit and is reported by the conversion workflow.
    """
