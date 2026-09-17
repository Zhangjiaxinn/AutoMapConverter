"""AutoMapConverter: multi-format road-map conversion and diagnostics."""

from .api import ConversionResult, MapFormat, convert, convert_and_diagnose, convert_batch_and_diagnose

__all__ = [
    "ConversionResult",
    "MapFormat",
    "convert",
    "convert_and_diagnose",
    "convert_batch_and_diagnose",
]
