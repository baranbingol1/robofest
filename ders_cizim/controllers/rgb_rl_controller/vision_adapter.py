"""Adapters that let non-Webots camera frames use the Webots-like vision path."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass(slots=True)
class RgbArrayCamera:
    frame: Any

    def getWidth(self) -> int:
        if hasattr(self.frame, "shape"):
            return int(self.frame.shape[1])
        if hasattr(self.frame, "size"):
            return int(self.frame.size[0])
        return len(self.frame[0]) if len(self.frame) else 0

    def getHeight(self) -> int:
        if hasattr(self.frame, "shape"):
            return int(self.frame.shape[0])
        if hasattr(self.frame, "size"):
            return int(self.frame.size[1])
        return len(self.frame)

    def getImage(self) -> Any:
        return self.frame


class RgbArrayCameraApi:
    @staticmethod
    def imageGetRed(image: Any, width: int, x: int, y: int) -> int:
        if hasattr(image, "getpixel"):
            return int(image.getpixel((x, y))[0])
        return int(image[y][x][0])

    @staticmethod
    def imageGetGreen(image: Any, width: int, x: int, y: int) -> int:
        if hasattr(image, "getpixel"):
            return int(image.getpixel((x, y))[1])
        return int(image[y][x][1])

    @staticmethod
    def imageGetBlue(image: Any, width: int, x: int, y: int) -> int:
        if hasattr(image, "getpixel"):
            return int(image.getpixel((x, y))[2])
        return int(image[y][x][2])
