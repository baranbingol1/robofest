"""Summaries for Webots RGB line-following smoke runs."""

from __future__ import annotations

import argparse
import glob
import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable, Sequence


@dataclass(frozen=True, slots=True)
class RunSummary:
    steps: int
    visible_ratio: float
    matched_target_ratio: float
    first_target_seen_step: int | None
    final_translation: tuple[float, float, float] | None
    final_goal_distance: float | None
    final_goal_margin: float | None
    travel_distance: float
    off_board: bool
    terminal_reason: str | None
    reached_goal: bool
    timed_out: bool
    lost_line: bool
    success: bool
    returned_start: bool = False
    sequence_complete: bool = False
    sequence_visited_colors: tuple[str, ...] = ()

    def passed_smoke_gate(
        self,
        *,
        min_visible_ratio: float = 0.90,
        require_target_lock: bool = True,
        require_goal_reached: bool = False,
        require_returned_start: bool = False,
        require_sequence_complete: bool = False,
        required_sequence_colors: Sequence[str] = (),
        min_goal_margin: float | None = None,
        min_travel_distance: float = 0.15,
    ) -> bool:
        if self.steps <= 0 or self.off_board:
            return False
        if self.lost_line:
            return False
        if self.timed_out:
            return False
        if self.visible_ratio < min_visible_ratio:
            return False
        if self.travel_distance < min_travel_distance:
            return False
        if require_target_lock and self.first_target_seen_step is None:
            return False
        if require_goal_reached and not self.reached_goal:
            return False
        if require_returned_start and not self.returned_start:
            return False
        if require_sequence_complete and not self.sequence_complete:
            return False
        if required_sequence_colors:
            visited = set(self.sequence_visited_colors)
            if any(color not in visited for color in required_sequence_colors):
                return False
        if min_goal_margin is not None:
            if self.final_goal_margin is None or self.final_goal_margin < min_goal_margin:
                return False
        return True


def _translation(record: dict[str, object]) -> tuple[float, float, float] | None:
    value = record.get("translation")
    if not isinstance(value, Sequence) or len(value) < 3:
        return None
    try:
        return (float(value[0]), float(value[1]), float(value[2]))
    except (TypeError, ValueError):
        return None


def _xy_distance(a: tuple[float, float, float], b: tuple[float, float, float]) -> float:
    return math.hypot(b[0] - a[0], b[1] - a[1])


def _goal_center(record: dict[str, object]) -> tuple[float, float] | None:
    value = record.get("goal_center")
    if not isinstance(value, Sequence) or len(value) < 2:
        return None
    try:
        return (float(value[0]), float(value[1]))
    except (TypeError, ValueError):
        return None


def _goal_radius(record: dict[str, object]) -> float | None:
    value = record.get("goal_radius")
    try:
        radius = float(value)
    except (TypeError, ValueError):
        return None
    return radius if radius >= 0.0 else None


def summarize_records(records: Iterable[dict[str, object]], *, board_half_extent: float = 1.0) -> RunSummary:
    rows = list(records)
    steps = len(rows)
    if steps == 0:
        return RunSummary(
            steps=0,
            visible_ratio=0.0,
            matched_target_ratio=0.0,
            first_target_seen_step=None,
            final_translation=None,
            final_goal_distance=None,
            final_goal_margin=None,
            travel_distance=0.0,
            off_board=False,
            terminal_reason=None,
            reached_goal=False,
            timed_out=False,
            lost_line=False,
            success=False,
            returned_start=False,
            sequence_complete=False,
            sequence_visited_colors=(),
        )

    visible_count = sum(1 for record in rows if bool(record.get("visible")))
    matched_count = sum(1 for record in rows if bool(record.get("matched_target")))
    first_target_seen_step = None
    for record in rows:
        if bool(record.get("target_seen")):
            try:
                first_target_seen_step = int(record.get("step", 0))
            except (TypeError, ValueError):
                first_target_seen_step = None
            break

    translations = [translation for translation in (_translation(record) for record in rows) if translation is not None]
    travel_distance = sum(_xy_distance(a, b) for a, b in zip(translations, translations[1:]))
    final_translation = translations[-1] if translations else None
    final_goal_distance = None
    final_goal_margin = None
    if final_translation is not None:
        for record in reversed(rows):
            center = _goal_center(record)
            radius = _goal_radius(record)
            if center is None or radius is None:
                continue
            final_goal_distance = math.hypot(final_translation[0] - center[0], final_translation[1] - center[1])
            final_goal_margin = radius - final_goal_distance
            break
    off_board = any(
        abs(translation[0]) > board_half_extent or abs(translation[1]) > board_half_extent
        for translation in translations
    )
    terminal_reason = None
    for record in rows:
        reason = record.get("terminal_reason")
        if isinstance(reason, str) and reason:
            terminal_reason = reason
            break
    reached_goal = terminal_reason == "reached_goal" or any(
        bool(record.get("reached_goal")) for record in rows
    )
    returned_start = terminal_reason == "returned_start" or any(
        bool(record.get("sequence_returned_start")) for record in rows
    )
    visited_colors: list[str] = []
    for record in rows:
        raw_visited = record.get("sequence_visited_colors")
        if isinstance(raw_visited, Sequence) and not isinstance(raw_visited, (str, bytes)):
            for color in raw_visited:
                if isinstance(color, str) and color not in visited_colors:
                    visited_colors.append(color)
        event = record.get("sequence_event")
        if isinstance(event, str) and event.startswith("reached_"):
            color = event.removeprefix("reached_")
            if color and color not in visited_colors:
                visited_colors.append(color)
    sequence_complete = returned_start or any(bool(record.get("sequence_complete")) for record in rows)
    reached_goal = reached_goal or returned_start
    timed_out = terminal_reason == "timeout"
    lost_line = terminal_reason == "lost_line"
    off_board = off_board or terminal_reason == "off_board"

    return RunSummary(
        steps=steps,
        visible_ratio=visible_count / steps,
        matched_target_ratio=matched_count / steps,
        first_target_seen_step=first_target_seen_step,
        final_translation=final_translation,
        final_goal_distance=final_goal_distance,
        final_goal_margin=final_goal_margin,
        travel_distance=travel_distance,
        off_board=off_board,
        terminal_reason=terminal_reason,
        reached_goal=reached_goal,
        timed_out=timed_out,
        lost_line=lost_line,
        success=(reached_goal or returned_start) and not off_board and not lost_line and not timed_out,
        returned_start=returned_start,
        sequence_complete=sequence_complete,
        sequence_visited_colors=tuple(visited_colors),
    )


def summarize_file(path: Path) -> RunSummary:
    return summarize_records(json.loads(path.read_text(encoding="utf-8")))


def _expand_paths(patterns: Iterable[str]) -> list[Path]:
    paths: list[Path] = []
    for pattern in patterns:
        matches = glob.glob(pattern)
        if matches:
            paths.extend(Path(match) for match in matches)
        else:
            paths.append(Path(pattern))
    return paths


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("paths", nargs="+", help="Step-log JSON files or glob patterns")
    parser.add_argument("--json", action="store_true", help="Print full JSON summaries")
    args = parser.parse_args()

    summaries = []
    for path in _expand_paths(args.paths):
        summary = summarize_file(path)
        row = {"path": str(path), **asdict(summary), "passed_smoke_gate": summary.passed_smoke_gate()}
        summaries.append(row)

    if args.json:
        print(json.dumps(summaries, indent=2, sort_keys=True))
        return

    for row in summaries:
        print(
            row["path"],
            f"steps={row['steps']}",
            f"visible={row['visible_ratio']:.3f}",
            f"matched={row['matched_target_ratio']:.3f}",
            f"first_lock={row['first_target_seen_step']}",
            f"distance={row['travel_distance']:.3f}",
            f"off_board={int(row['off_board'])}",
            f"terminal={row['terminal_reason'] or 'none'}",
            f"success={int(row['success'])}",
            f"pass={int(row['passed_smoke_gate'])}",
        )


if __name__ == "__main__":
    main()
