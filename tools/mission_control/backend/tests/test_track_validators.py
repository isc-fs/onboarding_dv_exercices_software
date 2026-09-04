"""Unit tests for track-name validators.

The path-traversal fix in #346 is the only thing standing between
"clean operator names" and "../../etc/passwd writes outside
TRACKS_DIR" — these tests pin every vector we care about so a future
regex tweak can't silently regress security.
"""

from __future__ import annotations

import os
import tempfile

import pytest

from track_validators import (
    TrackNameError,
    resolve_track_path,
    validate_track_name,
    validate_track_stem,
)


# ---------------------------------------------------------------------
# validate_track_name
# ---------------------------------------------------------------------


@pytest.mark.parametrize(
    "name",
    [
        "skidpad.csv",
        "track_001.csv",
        "A-track.csv",
        "A_B.C.csv",
        "a.csv",
        "ABCdef-_.123.csv",  # all allowed character classes
        "a" * 124 + ".csv",  # max length boundary (124 stem + 4 ext = 128)
    ],
)
def test_validate_track_name_accepts_clean_inputs(name: str) -> None:
    assert validate_track_name(name) == name


@pytest.mark.parametrize(
    "name",
    [
        # path traversal
        "../etc/passwd.csv",
        "../../etc/passwd.csv",
        "../../../etc/passwd",
        "/abs/path.csv",
        "foo/bar.csv",
        "foo\\bar.csv",
        # dotfile / `.` / `..`
        ".csv",
        "..csv",
        "...csv",
        ".hidden.csv",
        # wrong / missing extension
        "",
        "foo",
        "foo.txt",
        "foo.csv.bak",
        "foo.CSV",  # case-sensitive — refuse uppercase to keep the regex tight
        # shell metacharacters / whitespace / weird unicode
        "foo;rm.csv",
        "fo o.csv",
        "foo|bar.csv",
        "foo$.csv",
        "foo\nbar.csv",
        "foo\x00.csv",
        # over length cap (128 chars total = 124 stem + .csv)
        "a" * 125 + ".csv",
    ],
)
def test_validate_track_name_rejects_bad_inputs(name: str) -> None:
    with pytest.raises(TrackNameError):
        validate_track_name(name)


# ---------------------------------------------------------------------
# validate_track_stem
# ---------------------------------------------------------------------


@pytest.mark.parametrize(
    "stem",
    [
        "skidpad",
        "track_001",
        "A-track",
        "A_B.C",
        "a",
    ],
)
def test_validate_track_stem_accepts_clean_inputs(stem: str) -> None:
    assert validate_track_stem(stem) == stem


@pytest.mark.parametrize(
    "stem",
    [
        "",
        ".",
        "..",
        ".hidden",
        "foo/bar",
        "foo\\bar",
        "foo;rm",
        "../escape",
        "a" * 125,  # over 124-char cap
    ],
)
def test_validate_track_stem_rejects_bad_inputs(stem: str) -> None:
    with pytest.raises(TrackNameError):
        validate_track_stem(stem)


# ---------------------------------------------------------------------
# resolve_track_path
# ---------------------------------------------------------------------


def test_resolve_track_path_keeps_clean_name_inside(tmp_path) -> None:
    base = str(tmp_path)
    resolved = resolve_track_path("skidpad.csv", base)
    assert resolved == os.path.abspath(os.path.join(base, "skidpad.csv"))


def test_resolve_track_path_rejects_traversal(tmp_path) -> None:
    base = str(tmp_path)
    # Even if a name with `..` somehow got past `validate_track_name`
    # (e.g. regex weakened in a future PR), this is the second line of
    # defence. Pass a deliberately malicious name to confirm.
    with pytest.raises(TrackNameError):
        resolve_track_path("../escape.csv", base)


def test_resolve_track_path_handles_symlink_like_paths(tmp_path) -> None:
    base = str(tmp_path)
    # Absolute names should NOT escape — abspath collapses them but
    # the prefix check still applies.
    with pytest.raises(TrackNameError):
        resolve_track_path("/etc/passwd.csv", base)


def test_resolve_track_path_accepts_empty_segment_at_base(tmp_path) -> None:
    """`os.path.join(base, "")` resolves to `base` itself; the prefix
    check accepts that as a no-op (the directory itself, not a file
    inside it). The endpoint then fails on `os.path.exists` not
    matching a regular file — handled at the call site, not here."""
    base = str(tmp_path)
    resolved = resolve_track_path("", base)
    assert resolved == os.path.abspath(base)
