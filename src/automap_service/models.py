"""Shared, bounded request contract. File URLs are issued only by CICV."""
from typing import Literal
from urllib.parse import urlsplit

from pydantic import BaseModel, ConfigDict, Field, model_validator

Format = Literal["lanelet2", "opendrive", "osm"]
TERMINAL = {"COMPLETED", "FAILED", "CANCELLED", "TIMED_OUT"}


class TaskRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    taskId: str = Field(pattern=r"^[a-f0-9-]{36}$")
    sourceFormat: Format
    targetFormat: Format
    sourceUrl: str = Field(max_length=8192)
    uploadUrls: dict[str, str]
    diagnose: bool = True
    opendriveVersion: Literal["1.4", "1.5", "1.6", "1.7", "1.8"] = "1.6"
    timeoutSeconds: int = Field(default=1800, ge=30, le=3600)
    maxBytes: int = Field(default=104857600, ge=1, le=1073741824)

    @model_validator(mode="after")
    def validate_task(self):
        if self.sourceFormat == self.targetFormat:
            raise ValueError("Source and target formats must differ")
        if set(self.uploadUrls) != {"map", "report", "bundle"}:
            raise ValueError("Exactly map, report and bundle upload URLs are required")
        for url in [self.sourceUrl, *self.uploadUrls.values()]:
            parsed = urlsplit(url)
            if parsed.scheme not in {"http", "https"} or not parsed.hostname or parsed.username:
                raise ValueError("Invalid storage URL")
        return self
