"""Public Python API for AutoMapConverter."""

from .converter import (
    ConversionResult,
    MapFormat,
    convert,
    convert_and_diagnose,
    convert_batch_and_diagnose,
)

__all__ = [
    "ConversionResult",
    "MapFormat",
    "convert",
    "convert_and_diagnose",
    "convert_batch_and_diagnose",
]
