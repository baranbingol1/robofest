#!/usr/bin/env python3
from __future__ import annotations

import html
import re
import subprocess
import sys
import urllib.error
import urllib.request
from datetime import date
from pathlib import Path


ROOT = Path(__file__).resolve().parent
WORKSPACE_ROOT = ROOT.parent.parent
WEBOTS_REPO = WORKSPACE_ROOT / "webots-docs"
OUTPUT = ROOT / "webots-python-llms-full.txt"

GUIDE_FILES = [
    "docs/guide/using-python.md",
    "docs/guide/controller-programming.md",
    "docs/guide/controller-start-up.md",
    "docs/guide/cpp-java-python.md",
    "docs/guide/compiling-controllers-in-a-terminal.md",
    "docs/guide/supervisor-programming.md",
    "docs/guide/running-extern-robot-controllers.md",
]

REFERENCE_OVERVIEW_FILES = [
    "docs/reference/nodes-and-api-functions.md",
    "docs/reference/device.md",
    "docs/reference/robot.md",
    "docs/reference/supervisor.md",
    "docs/reference/execution-scheme.md",
    "docs/reference/other-apis.md",
    "docs/reference/utility-functions.md",
    "docs/reference/motion.md",
    "docs/reference/motion-functions.md",
]

REFERENCE_DEVICE_FILES = [
    "docs/reference/accelerometer.md",
    "docs/reference/altimeter.md",
    "docs/reference/brake.md",
    "docs/reference/camera.md",
    "docs/reference/charger.md",
    "docs/reference/compass.md",
    "docs/reference/connector.md",
    "docs/reference/display.md",
    "docs/reference/distancesensor.md",
    "docs/reference/emitter.md",
    "docs/reference/gps.md",
    "docs/reference/gyro.md",
    "docs/reference/inertialunit.md",
    "docs/reference/joystick.md",
    "docs/reference/keyboard.md",
    "docs/reference/led.md",
    "docs/reference/lidar.md",
    "docs/reference/lightsensor.md",
    "docs/reference/linearmotor.md",
    "docs/reference/motor.md",
    "docs/reference/mouse.md",
    "docs/reference/muscle.md",
    "docs/reference/pen.md",
    "docs/reference/positionsensor.md",
    "docs/reference/propeller.md",
    "docs/reference/radar.md",
    "docs/reference/rangefinder.md",
    "docs/reference/receiver.md",
    "docs/reference/recognition.md",
    "docs/reference/rotationalmotor.md",
    "docs/reference/speaker.md",
    "docs/reference/touchsensor.md",
    "docs/reference/track.md",
    "docs/reference/vacuumgripper.md",
]

ALL_FILES = GUIDE_FILES + REFERENCE_OVERVIEW_FILES + REFERENCE_DEVICE_FILES

DIRECTIVE_PREFIXES = (
    "%tab-component",
    "%tab-end",
    "%figure",
    "%chart",
    "%end",
)

LANGUAGE_TABS = {"C", "C++", "Python", "Java", "MATLAB"}


def run_git(*args: str) -> str:
    return subprocess.check_output(
        ["git", "-C", str(WEBOTS_REPO), *args],
        text=True,
        stderr=subprocess.DEVNULL,
    )


def repo_commit() -> str:
    return run_git("rev-parse", "HEAD").strip()


def fetch_doc(path: str, commit: str) -> str:
    local_path = WEBOTS_REPO / path
    if local_path.exists():
        return local_path.read_text(encoding="utf-8")

    url = f"https://raw.githubusercontent.com/cyberbotics/webots/{commit}/{path}"
    with urllib.request.urlopen(url, timeout=30) as response:
        return response.read().decode("utf-8")


def collapse_blank_lines(text: str) -> str:
    text = re.sub(r"\n{3,}", "\n\n", text)
    return text.strip() + "\n"


def normalize_inline_markup(line: str) -> str:
    line = re.sub(r"!\[[^\]]*]\([^)]*\)", "", line)
    line = re.sub(r"\[([^\]]+)]\(([^)]+)\)", r"\1 (\2)", line)
    line = re.sub(r"`{3,}", "```", line)
    return html.unescape(line.rstrip())


def keep_python_tabs(markdown: str) -> str:
    output: list[str] = []
    current_tab: str | None = None

    for raw_line in markdown.splitlines():
        stripped = raw_line.strip()

        if stripped.startswith('%tab "'):
            match = re.match(r'%tab "([^"]+)"', stripped)
            current_tab = match.group(1) if match else None
            continue
        if stripped == "%tab-end":
            current_tab = None
            continue
        if stripped in DIRECTIVE_PREFIXES or any(stripped.startswith(prefix) for prefix in DIRECTIVE_PREFIXES):
            continue

        if current_tab is not None and current_tab in LANGUAGE_TABS and current_tab != "Python":
            continue

        output.append(normalize_inline_markup(raw_line))

    return collapse_blank_lines("\n".join(output))


def build_header(commit: str) -> str:
    sources = "\n".join(f"- `{path}`" for path in ALL_FILES)
    return (
        "# Webots Python llms-full\n\n"
        "> Flattened, LLM-oriented context bundle for writing Webots controllers in Python. "
        f"Generated on {date.today().isoformat()} from the official Cyberbotics Webots docs at commit `{commit}`.\n\n"
        "This file follows the common `llms-full.txt` pattern used alongside `llms.txt`: "
        "a single readable text/markdown bundle assembled from curated documentation pages. "
        "It is not an official `llmstxt.org` file format, but a practical companion artifact for agents and coding assistants.\n\n"
        "## Included Sources\n\n"
        f"{sources}\n\n"
        "## Normalization Rules\n\n"
        "- Preserved explanatory prose and API signatures from the official docs.\n"
        "- Reduced multi-language tabbed examples to the Python tab when one exists.\n"
        "- Removed site template directives and image-only markup.\n"
        "- Kept markdown headings and code fences so the file stays both human- and LLM-readable.\n"
    )


def main() -> int:
    if not WEBOTS_REPO.exists():
        print(f"Missing Webots repo at {WEBOTS_REPO}", file=sys.stderr)
        return 1

    commit = repo_commit()
    sections = [build_header(commit)]
    failures: list[str] = []

    for path in ALL_FILES:
        try:
            raw = fetch_doc(path, commit)
        except (subprocess.CalledProcessError, urllib.error.URLError) as exc:
            failures.append(f"{path}: {exc}")
            continue

        cleaned = keep_python_tabs(raw)
        sections.append(f"\n## Source: {path}\n\n{cleaned}")

    if failures:
        failure_block = "\n".join(f"- {item}" for item in failures)
        sections.insert(
            1,
            "\n## Missing Sources\n\n"
            "The following pages could not be retrieved during generation:\n"
            f"{failure_block}\n",
        )

    OUTPUT.write_text("\n".join(sections).rstrip() + "\n", encoding="utf-8")
    print(f"Wrote {OUTPUT}")
    return 0 if not failures else 2


if __name__ == "__main__":
    raise SystemExit(main())
