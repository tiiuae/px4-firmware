#!/usr/bin/env python3
import os
import re
import json
from pathlib import Path

ROOT = Path(".")
SRC_MODULES = ROOT / "src/modules"
SRC_DRIVERS = ROOT / "src/drivers"

OUTPUT_JSON = ROOT / "px4_graph.json"
OUTPUT_MD = ROOT / "px4_graph.md"

# -------------------------------------------------------------------------
# REGEXES
# -------------------------------------------------------------------------
RE_SUB = re.compile(r"uORB::Subscription(?:Data)?<\s*([\w_]+)\s*>")
RE_PUB = re.compile(r"uORB::Publication(?:Multi)?<\s*([\w_]+)\s*>")
RE_ADVERTISE = re.compile(r"orb_advertise\w*\(\s*&?([\w_]+)")
RE_PUBLISH = re.compile(r"orb_publish\w*\(\s*&?([\w_]+)")

# -------------------------------------------------------------------------
# DATA STRUCTURES
# -------------------------------------------------------------------------

drivers = {}
modules = {}

# -------------------------------------------------------------------------
# SCANNING UTILITIES
# -------------------------------------------------------------------------

def get_first_level_dirs(parent: Path):
    """Return only the immediate subdirectories (one level deep)."""
    return [d for d in parent.iterdir() if d.is_dir()]


def read_file(path: Path) -> str:
    try:
        return path.read_text(errors="ignore")
    except:
        return ""


def extract_topics(text: str):
    """Extract uORB subscriptions and publications from file text."""
    subs = set(RE_SUB.findall(text))
    pubs = set(RE_PUB.findall(text))
    pubs |= set(RE_ADVERTISE.findall(text))
    pubs |= set(RE_PUBLISH.findall(text))
    return subs, pubs


def detect_physical_input(driver_name: str):
    """Convert driver folder name into a human-readable external input."""
    name = driver_name.lower()
    if "gps" in name:
        return "GPS"
    if "imu" in name or "mpu" in name or "icm" in name or "bmi" in name:
        return "IMU"
    if "mag" in name or "hmc" in name or "lis3mdl" in name or "magnet" in name:
        return "Magnetometer"
    if "baro" in name or "ms5611" in name or "bmp" in name:
        return "Barometer"
    if "airspeed" in name:
        return "Airspeed"
    if "range" in name or "lidar" in name or "vl53" in name or "tfmini" in name:
        return "Rangefinder / Lidar"
    return driver_name  # fallback, raw driver name


# -------------------------------------------------------------------------
# SCAN DRIVERS
# -------------------------------------------------------------------------

def scan_drivers():
    for d in get_first_level_dirs(SRC_DRIVERS):
        drv = d.name
        drivers[drv] = {
            "physical_input": detect_physical_input(drv),
            "publishes": set(),
            "path": str(d)
        }

        for root, _, files in os.walk(d):
            for f in files:
                if f.endswith((".cpp", ".c", ".hpp", ".h")):
                    text = read_file(Path(root) / f)
                    _, pubs = extract_topics(text)
                    drivers[drv]["publishes"].update(pubs)


# -------------------------------------------------------------------------
# SCAN MODULES
# -------------------------------------------------------------------------

def scan_modules():
    for m in get_first_level_dirs(SRC_MODULES):
        mod = m.name
        modules[mod] = {
            "sub": set(),
            "pub": set(),
            "drivers_used": set(),
            "description": "",
            "path": str(m),
        }

        for root, _, files in os.walk(m):
            for f in files:
                if f.endswith((".cpp", ".c", ".hpp", ".h")):
                    fp = Path(root) / f
                    text = read_file(fp)
                    subs, pubs = extract_topics(text)
                    modules[mod]["sub"].update(subs)
                    modules[mod]["pub"].update(pubs)

                    # detect which drivers feed this module
                    for drv in drivers:
                        if drv.lower() in text.lower():
                            modules[mod]["drivers_used"].add(drv)


# -------------------------------------------------------------------------
# GENERATE MARKDOWN
# -------------------------------------------------------------------------

def make_markdown():
    md = []

    md.append("# PX4 uORB Graph\n")
    md.append("## **Drivers → uORB Publications**\n")
    md.append("| Driver | Physical Input | Publishes | Path |")
    md.append("|--------|----------------|-----------|------|")

    for drv, d in sorted(drivers.items()):
        pubs = ", ".join(sorted(d["publishes"]))
        md.append(
            f"| `{drv}` | `{d['physical_input']}` | `{pubs}` | `{d['path']}` |"
        )

    md.append("\n\n## **Modules → Subscriptions / Publications / Driver Inputs**\n")
    md.append("| Module | Subscribes | Publishes | External Inputs | Path |")
    md.append("|--------|------------|-----------|-----------------|------|")

    for mod, d in sorted(modules.items()):
        subs = ", ".join(sorted(d["sub"]))
        pubs = ", ".join(sorted(d["pub"]))
        drv = ", ".join(sorted(d["drivers_used"]))
        md.append(
            f"| `{mod}` | `{subs}` | `{pubs}` | `{drv}` | `{d['path']}` |"
        )

    return "\n".join(md)


# -------------------------------------------------------------------------
# MAIN
# -------------------------------------------------------------------------

def main():
    print("Scanning drivers...")
    scan_drivers()

    print("Scanning modules...")
    scan_modules()

    # convert everything to JSON-serializable types
    json_output = {
        "drivers": {
            k: {
                "physical_input": v["physical_input"],
                "publishes": sorted(v["publishes"]),
                "path": v["path"],
            }
            for k, v in drivers.items()
        },
        "modules": {
            k: {
                "sub": sorted(v["sub"]),
                "pub": sorted(v["pub"]),
                "drivers_used": sorted(v["drivers_used"]),
                "path": v["path"],
            }
            for k, v in modules.items()
        },
    }

    OUTPUT_JSON.write_text(json.dumps(json_output, indent=2))
    OUTPUT_MD.write_text(make_markdown())

    print("Done!")
    print(f" → JSON written to: {OUTPUT_JSON}")
    print(f" → Markdown written to: {OUTPUT_MD}")


if __name__ == "__main__":
    main()
