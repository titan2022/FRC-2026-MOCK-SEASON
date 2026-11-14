#!/usr/bin/env python3
"""Check the Elastic layout against the robot's published topics and tuning fields."""

import itertools
import json
import math
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
LAYOUT = ROOT / "dashboard/elastic-layout.json"
TYPES = {"Large Text Display", "Text Display", "ComboBox Chooser", "Boolean Box",
         "Number Bar", "Voltage View", "Camera Stream"}
BOUNDS = {"Driver": (1280, 512), "Auto setup": (1280, 640),
          "Diagnostics": (1280, 384), "Cameras": (1280, 384)}


def validate(layout):
    errors = []
    driver = (ROOT / "src/main/java/frc/robot/dashboard/DriverDashboard.java").read_text()
    settings = (ROOT / "src/main/java/frc/robot/auto/AutoSettings.java").read_text()
    manager = (ROOT / "src/main/java/frc/robot/auto/AutoManager.java").read_text()
    driver_keys = set(re.findall(r'\b(?:text|number|flag)\("([^"]+)"', driver))
    driver_keys.update("Preset " + button for button in "XYAB")
    topics = {"/SmartDashboard/Driver/" + key for key in driver_keys}
    topics.update("/SmartDashboard/" + key for key in re.findall(
        r'SmartDashboard\.put(?:String|Number|Boolean|Data)\("([^"]+)"', manager))
    tuning_keys = set(re.findall(r'values\.put\("([^"]+)"', settings))
    tuning_topics = {"/SmartDashboard/Auto/Tuning/" + key for key in tuning_keys}
    topics.update(tuning_topics)
    topics.update(("/CameraPublisher/USB Camera 0", "/CameraPublisher/USB Camera 1"))
    limit_match = re.search(r'MAX_DRIVE_POWER\s*=\s*([0-9.]+)', settings)
    if not limit_match:
        errors.append("Cannot read AutoSettings.MAX_DRIVE_POWER")
        drive_limit = 0.60
    else:
        drive_limit = float(limit_match.group(1))
    drive_bounds = f"-{drive_limit:.2f} to {drive_limit:.2f}"
    if layout.get("version") != 1.0:
        errors.append("Expected Elastic layout version 1.0")
    grid = layout.get("grid_size")
    if grid != 64:
        errors.append("Expected 64-pixel grid")
    tabs = layout.get("tabs", [])
    if [tab.get("name") for tab in tabs] != list(BOUNDS):
        errors.append("Unexpected tab names/order")
    count = 0
    edited_topics = []
    for tab in tabs:
        name = tab.get("name")
        if name not in BOUNDS:
            continue
        width, height = BOUNDS[name]
        content = tab.get("grid_layout", {})
        if content.get("layouts") != []:
            errors.append(f"{name}: unexpected nested layout")
        widgets = content.get("containers", [])
        count += len(widgets)
        valid_rects = []
        for item in widgets:
            title = item.get("title", "<untitled>")
            kind = item.get("type")
            props = item.get("properties", {})
            topic = props.get("topic")
            if not isinstance(title, str) or not title:
                errors.append(f"{name}: widget title missing")
            if kind not in TYPES:
                errors.append(f"{title}: unsupported widget type {kind!r}")
            if topic not in topics:
                errors.append(f"{title}: topic has no matching Java publisher: {topic}")
            if props.get("period") != 0.1:
                errors.append(f"{title}: expected 0.1-second update period")
            rect = [item.get(key) for key in ("x", "y", "width", "height")]
            if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in rect):
                errors.append(f"{title}: invalid rectangle")
                continue
            x, y, w, h = rect
            if x < 0 or y < 0 or w <= 0 or h <= 0 or x + w > width or y + h > height:
                errors.append(f"{title}: rectangle exceeds {name} bounds")
            if any(v % 64 != 0 for v in rect):
                errors.append(f"{title}: rectangle does not align with the grid")
            valid_rects.append((title, x, y, w, h))
            if topic in tuning_topics:
                edited_topics.append(topic)
                if kind != "Text Display" or props.get("show_submit_button") is not True:
                    errors.append(f"{title}: tuning field needs a submitted numeric editor")
                if props.get("data_type") != "double":
                    errors.append(f"{title}: tuning input must declare data_type double")
                if topic.rsplit("/", 1)[-1] in (
                        "Cross drive power", "Score approach power", "Score exit power"):
                    if drive_bounds not in title:
                        errors.append(f"{title}: drive-power label must contain {drive_bounds}")
        for a, b in itertools.combinations(valid_rects, 2):
            _, ax, ay, aw, ah = a
            _, bx, by, bw, bh = b
            if ax < bx + bw and bx < ax + aw and ay < by + bh and by < ay + ah:
                errors.append(f"{name}: widgets overlap: {a[0]} / {b[0]}")
    if count != 57:
        errors.append(f"Expected 57 widgets, found {count}")
    if len(tuning_keys) != 14:
        errors.append(f"Expected 14 tuning entries in AutoSettings, found {len(tuning_keys)}")
    if set(edited_topics) != tuning_topics or len(edited_topics) != len(tuning_topics):
        errors.append("Each tuning entry must appear exactly once as an editor")
    return errors, count, len(tuning_topics), drive_bounds


def main():
    path = Path(sys.argv[1]) if len(sys.argv) > 1 else LAYOUT
    errors, count, tuning_count, bounds = validate(json.loads(path.read_text()))
    if errors:
        for message in errors:
            print("FAIL: " + message, file=sys.stderr)
        return 1
    print(f"PASS: 4 tabs, {count} non-overlapping widgets; all {tuning_count} tuning fields "
          "bound as doubles; topics match Java publishers.")
    print(f"Drive power labels match AutoSettings.MAX_DRIVE_POWER: {bounds}.")
    print("Camera topics are optional placeholders; camera availability is not asserted.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
