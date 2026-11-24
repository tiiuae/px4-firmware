"""Guard rails for shell commands sent to the board."""

import re

# Commands that need explicit user confirmation (confirm=True) before they are sent.
CONFIRM_PATTERNS = [
    (r"^param\s+(set|set-default|reset|reset_all|reset_nostart|import|load|save|touch|select)\b",
     "modifies parameters"),
    (r"^(motor_test|actuator_test|esc_calib|pwm|dshot)\b", "can spin motors or move actuators"),
    (r"^commander\s+(arm|takeoff|land|mode|calibrate|failsafe|lockdown|transition|set_ekf_origin)\b",
     "changes vehicle state"),
    (r"^(rm|rmdir|mv|cp|mkdir|mkfatfs|mkfs|mtd|bl_update|dd|umount|mount|flash_erase|truncate)\b",
     "modifies storage"),
    (r"^(sd_bench|sd_stress)\b", "writes to storage"),
    (r">", "redirects output into a file"),
]

# Commands that must go through a dedicated tool.
BLOCKED_PATTERNS = [
    (r"^(reboot|shutdown|poweroff)\b", "use the `reboot` tool instead"),
]

_SPLIT_RE = re.compile(r"\s*(?:;|&&|\|\||\|)\s*")


def check_command(command: str, extra_confirm_patterns=()) -> tuple[list[str], list[str]]:
    """Return (blocked_reasons, confirm_reasons) for a shell command line."""
    blocked, confirm = [], []
    confirm_patterns = CONFIRM_PATTERNS + [(p, "matches configured confirm pattern") for p in extra_confirm_patterns]
    for part in _SPLIT_RE.split(command.strip()):
        if not part:
            continue
        for pattern, reason in BLOCKED_PATTERNS:
            if re.search(pattern, part):
                blocked.append(f"'{part}': {reason}")
        for pattern, reason in confirm_patterns:
            if re.search(pattern, part):
                confirm.append(f"'{part}': {reason}")
    return blocked, confirm
