"""
Pure-function validators for track filenames + paths.

Extracted from main.py (#346 path-traversal fix) into a standalone
module so the CI test suite can exercise them without standing up the
whole FastAPI app + sim connection. main.py imports from here; the
behaviour is identical.

The pattern + path resolution is the only thing standing between
"clean operator names" and "../../etc/passwd.csv writes outside
TRACKS_DIR" — keep it boring and well-tested.
"""

from __future__ import annotations

import os
import re
from typing import Optional


_TRACK_NAME_RE = re.compile(r"^[A-Za-z0-9._-]+\.csv$")
_TRACK_STEM_RE = re.compile(r"^[A-Za-z0-9._-]+$")


class TrackNameError(ValueError):
    """Raised when a track name fails the path-traversal check.

    Catchers (the FastAPI endpoint) translate this to HTTPException(400);
    `validate_track_name` and `validate_track_stem` raise this so the
    test suite can assert on the failure modes directly without pulling
    in `fastapi` for a pure-function check.
    """


def validate_track_name(name: str) -> str:
    """Validate a `<stem>.csv` track filename.

    Allowed: letters / digits / `._-`, must end in `.csv`, length in
    [1, 128]. Rejected: empty, `.`, `..`, dotfile stems, anything with
    `/`, `\\`, whitespace, shell metacharacters, etc.

    Returns the validated name unchanged so callers can keep using
    `os.path.join(TRACKS_DIR, name)`. Raises `TrackNameError` on
    invalid input.
    """
    if not name or len(name) > 128:
        raise TrackNameError("Invalid track name")
    if not _TRACK_NAME_RE.fullmatch(name):
        raise TrackNameError(
            "Invalid track name (allowed chars: A-Za-z0-9._-, must end with .csv)"
        )
    stem = name[:-4]  # strip ".csv"
    if stem in (".", "..") or stem.startswith("."):
        raise TrackNameError("Invalid track name")
    return name


def validate_track_stem(stem: str) -> str:
    """Same as `validate_track_name` but for the `.csv`-less stem.
    Used by `track_generate` which appends `.csv` itself.
    """
    if not stem or len(stem) > 124:  # leave 4 chars for ".csv"
        raise TrackNameError("Invalid track name")
    if not _TRACK_STEM_RE.fullmatch(stem):
        raise TrackNameError(
            "Invalid track name (allowed chars: A-Za-z0-9._-)"
        )
    if stem in (".", "..") or stem.startswith("."):
        raise TrackNameError("Invalid track name")
    return stem


def resolve_track_path(name: str, tracks_dir: str) -> str:
    """Resolve `<tracks_dir>/<name>` and assert the result stays inside
    `tracks_dir`. Belt-and-braces with `validate_track_name` — if the
    regex is ever weakened (e.g. someone adds a colon to the allowed
    set), this still keeps reads/writes scoped to the tracks directory.

    Raises `TrackNameError` if the resolved path escapes `tracks_dir`.
    """
    base = os.path.abspath(tracks_dir)
    target = os.path.abspath(os.path.join(base, name))
    if not (target == base or target.startswith(base + os.sep)):
        raise TrackNameError("Invalid track path")
    return target
