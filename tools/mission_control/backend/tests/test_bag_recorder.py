"""Unit tests for tools/mission_control/backend/bag_recorder.py (#465 v2).

bag_recorder is now a thin client over the /bag_recorder/{start,stop}
ROS services hosted by dv_pipeline_stack's bag_recorder_node. The
tests mock out the rclpy plumbing so the suite stays pure-Python.

Subprocess management lives in pipeline/bag_recorder_node/recorder.py
and has its own test suite there. This file tests just the
ROS-service-client wrapper.
"""
from __future__ import annotations

import os
import sys
from datetime import datetime, timezone
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), os.pardir))

import bag_recorder as br  # noqa: E402


# ----- compose_bag_name --------------------------------------------------

def test_compose_bag_name_sanitises_and_orders_segments():
    when = datetime(2026, 5, 14, 15, 30, 22, tzinfo=timezone.utc)
    out = br.compose_bag_name("trackdrive", "TrainingMap", now=when)
    assert out == "trackdrive_TrainingMap_20260514_153022"


def test_compose_bag_name_strips_csv_extension():
    when = datetime(2026, 5, 14, 15, 30, 22, tzinfo=timezone.utc)
    out = br.compose_bag_name("trackdrive", "track_20260512_151240.csv", now=when)
    assert out == "trackdrive_track_20260512_151240_20260514_153022"
    assert ".csv" not in out


def test_compose_bag_name_replaces_unsafe_chars():
    when = datetime(2026, 5, 14, 15, 30, 22, tzinfo=timezone.utc)
    out = br.compose_bag_name("track drive!", "../etc/passwd", now=when)
    assert " " not in out
    assert "/" not in out
    assert ".." not in out
    assert "!" not in out


def test_compose_bag_name_fallback_when_track_missing():
    when = datetime(2026, 5, 14, 15, 30, 22, tzinfo=timezone.utc)
    out = br.compose_bag_name("trackdrive", None, now=when)
    assert out == "trackdrive_no-track_20260514_153022"


def test_compose_bag_name_fallback_when_event_unknown():
    when = datetime(2026, 5, 14, 15, 30, 22, tzinfo=timezone.utc)
    out = br.compose_bag_name("", "TrainingMap", now=when)
    assert out == "unknown_TrainingMap_20260514_153022"


# ----- request_start / request_stop --------------------------------------
#
# These exercise the rclpy service-client path. We monkey-patch the
# lazy-init `_ensure_clients` to drop in mocks that match the rclpy
# service-client surface (wait_for_service, call_async, the response
# fields the wrapper reads). _reset_clients_for_test clears the module
# globals between tests so each one starts from a clean slate.


def _make_fake_bridge():
    """Return a stand-in for the RosBridge singleton."""
    bridge = MagicMock()
    bridge._node = MagicMock()
    return bridge


def _install_fake_clients(monkeypatch, start_resp, stop_resp,
                          start_reachable=True, stop_reachable=True,
                          start_call_returns_none=False,
                          stop_call_returns_none=False):
    """Monkey-patch bag_recorder's lazy client init to inject mocks.

    `start_resp`/`stop_resp` are SimpleNamespace objects with the
    response fields the wrapper reads (ok / state / bag_path / error).
    None on `*_call_returns_none` simulates a future that timed out.
    """
    br._reset_clients_for_test()

    fake_start_client = MagicMock()
    fake_start_client.wait_for_service = MagicMock(return_value=start_reachable)
    fake_stop_client = MagicMock()
    fake_stop_client.wait_for_service = MagicMock(return_value=stop_reachable)

    # Stand-in Srv types whose .Request() returns a settable object.
    class _FakeStartReq:
        def __init__(self):
            self.bag_name = ""

    class _FakeStopReq:
        pass

    class _FakeStartSrv:
        Request = _FakeStartReq

    class _FakeStopSrv:
        Request = _FakeStopReq

    def _ensure(_bridge):
        br._start_client = fake_start_client
        br._stop_client = fake_stop_client
        br._start_srv_type = _FakeStartSrv
        br._stop_srv_type = _FakeStopSrv

    monkeypatch.setattr(br, "_ensure_clients", _ensure)

    # `_call_sync` does the future-wait dance. Mock it directly — the
    # actual rclpy future plumbing is library-internal, not interesting
    # for the wrapper's contract.
    def _fake_call_sync(client, request, timeout_s):
        if client is fake_start_client:
            return None if start_call_returns_none else start_resp
        if client is fake_stop_client:
            return None if stop_call_returns_none else stop_resp
        raise AssertionError("unexpected client passed to _call_sync")

    monkeypatch.setattr(br, "_call_sync", _fake_call_sync)


def test_request_start_happy_path(monkeypatch):
    start_resp = SimpleNamespace(
        ok=True, state="recording",
        bag_path="/bags/trackdrive_X_20260514_153022",
        error="",
    )
    _install_fake_clients(monkeypatch, start_resp, stop_resp=None)

    out = br.request_start(_make_fake_bridge(), "trackdrive_X_20260514_153022")

    assert out["ok"] is True
    assert out["state"] == "recording"
    assert out["name"] == "trackdrive_X_20260514_153022"
    assert out["path"] == "/bags/trackdrive_X_20260514_153022"
    assert out["error"] == ""


def test_request_start_propagates_disk_full(monkeypatch):
    # bag_recorder_node returns ok=False, state="failed", with an error
    # explaining the disk situation. The client must surface that
    # verbatim so the UI can show it.
    start_resp = SimpleNamespace(
        ok=False, state="failed", bag_path="",
        error="only 3 GiB free on /bags (need ≥10 GiB)",
    )
    _install_fake_clients(monkeypatch, start_resp, stop_resp=None)

    out = br.request_start(_make_fake_bridge(), "x_y_20260514_153022")

    assert out["ok"] is False
    assert out["state"] == "failed"
    assert "3 GiB" in out["error"]


def test_request_start_handles_unreachable_service(monkeypatch):
    _install_fake_clients(
        monkeypatch, start_resp=None, stop_resp=None,
        start_reachable=False,
    )

    out = br.request_start(
        _make_fake_bridge(), "x_y_20260514_153022", timeout_s=0.1,
    )

    assert out["ok"] is False
    assert out["state"] == "failed"
    assert "not reachable" in out["error"]


def test_request_start_handles_timeout(monkeypatch):
    _install_fake_clients(
        monkeypatch, start_resp=None, stop_resp=None,
        start_call_returns_none=True,
    )

    out = br.request_start(
        _make_fake_bridge(), "x_y_20260514_153022", timeout_s=0.1,
    )

    assert out["ok"] is False
    assert out["state"] == "failed"
    assert "timed out" in out["error"]


def test_request_stop_happy_path(monkeypatch):
    stop_resp = SimpleNamespace(
        ok=True, state="stopped",
        bag_path="/bags/trackdrive_X_20260514_153022",
        error="",
    )
    _install_fake_clients(monkeypatch, start_resp=None, stop_resp=stop_resp)

    out = br.request_stop(_make_fake_bridge())

    assert out["ok"] is True
    assert out["state"] == "stopped"
    assert out["path"] == "/bags/trackdrive_X_20260514_153022"
    assert out["error"] == ""


def test_request_stop_idempotent_when_nothing_active(monkeypatch):
    stop_resp = SimpleNamespace(ok=True, state="none", bag_path="", error="")
    _install_fake_clients(monkeypatch, start_resp=None, stop_resp=stop_resp)

    out = br.request_stop(_make_fake_bridge())

    assert out["ok"] is True
    assert out["state"] == "none"
    assert out["path"] == ""


def test_request_stop_surfaces_failure(monkeypatch):
    stop_resp = SimpleNamespace(
        ok=False, state="failed",
        bag_path="/tmp/ifssim_bag_active/x_y_20260514_153022",
        error="staging→final move failed: cross-device link not permitted",
    )
    _install_fake_clients(monkeypatch, start_resp=None, stop_resp=stop_resp)

    out = br.request_stop(_make_fake_bridge())

    assert out["ok"] is False
    assert out["state"] == "failed"
    assert "move failed" in out["error"]


def test_request_stop_handles_timeout(monkeypatch):
    _install_fake_clients(
        monkeypatch, start_resp=None, stop_resp=None,
        stop_call_returns_none=True,
    )

    out = br.request_stop(_make_fake_bridge(), timeout_s=0.1)

    assert out["ok"] is False
    assert out["state"] == "failed"
    assert "timed out" in out["error"]


# ----- #498: auto_pull_and_clean -----------------------------------------
#
# These tests mock the docker Python SDK so the suite stays pure-
# Python — no real daemon, no real container. They cover the path
# auto_pull_and_clean takes in production: stream a bag tarball via
# container.get_archive() into a tempdir, then clean the volume copy
# via container.exec_run().

def _make_tarball_bytes(bag_name: str, files: dict) -> bytes:
    """Build a tarball laid out the way docker.get_archive returns
    one for a directory: top-level entry is `<bag_name>/`, contents
    are children inside that directory.
    """
    import io as _io
    import tarfile as _tar
    buf = _io.BytesIO()
    with _tar.open(fileobj=buf, mode="w") as tar:
        # Directory entry
        d = _tar.TarInfo(name=bag_name)
        d.type = _tar.DIRTYPE
        d.mode = 0o755
        tar.addfile(d)
        for fname, data in files.items():
            info = _tar.TarInfo(name=f"{bag_name}/{fname}")
            info.size = len(data)
            tar.addfile(info, fileobj=_io.BytesIO(data))
    return buf.getvalue()


def _fake_container_factory(get_archive=None, exec_run=None):
    """Build a fake `docker.containers.get()` result."""
    c = MagicMock(name="FakeContainer")
    c.get_archive = MagicMock(side_effect=get_archive or (
        lambda path: (iter([b""]), {"size": 0})))
    c.exec_run = MagicMock(side_effect=exec_run or (
        lambda cmd, **kw: (0, b"")))
    return c


def _patch_docker_sdk(monkeypatch, container=None, ping_ok=True,
                      module_missing=False, from_env_raises=None):
    """Install a fake `docker` module in sys.modules so the SDK import
    inside bag_recorder._get_docker_client() picks it up. Returns the
    fake client so tests can introspect calls.
    """
    if module_missing:
        # Simulate ImportError on `import docker`.
        monkeypatch.setitem(sys.modules, "docker", None)
        br._reset_docker_probe_cache_for_test()
        return None

    fake_docker = MagicMock(name="FakeDockerModule")
    fake_client = MagicMock(name="FakeClient")
    if from_env_raises:
        fake_docker.from_env = MagicMock(side_effect=from_env_raises)
    else:
        fake_docker.from_env = MagicMock(return_value=fake_client)
        if ping_ok:
            fake_client.ping = MagicMock(return_value=True)
        else:
            fake_client.ping = MagicMock(side_effect=ConnectionError("no socket"))
        if container is not None:
            fake_client.containers.get = MagicMock(return_value=container)
        else:
            fake_client.containers.get = MagicMock(
                side_effect=Exception("No such container"))

    monkeypatch.setitem(sys.modules, "docker", fake_docker)
    br._reset_docker_probe_cache_for_test()
    return fake_client


def _env(monkeypatch, **kw):
    """Set env vars + reset the docker client cache."""
    monkeypatch.setenv("IFSSIM_BAG_AUTO_PULL", kw.pop("IFSSIM_BAG_AUTO_PULL", "1"))
    monkeypatch.setenv(
        "DV_PIPELINE_STACK_CONTAINER",
        kw.pop("DV_PIPELINE_STACK_CONTAINER", "ifssim-dv_pipeline_stack-1"),
    )
    br._reset_docker_probe_cache_for_test()


def test_auto_pull_happy_path(monkeypatch, tmp_path):
    _env(monkeypatch)
    # Point _HOST_BAGS_DIR at a tmp dir so the extraction is real
    # (we test that files actually land).
    monkeypatch.setattr(br, "_HOST_BAGS_DIR", str(tmp_path))

    name = "trackdrive_skidpad_20260514_191500"
    tarball = _make_tarball_bytes(name, {
        "metadata.yaml": b"version: 5\nstorage_id: mcap\n",
        "bag_0.mcap": b"fake_mcap_data",
    })
    # Chunk the bytes to mimic a streaming response.
    def _get_archive(path):
        assert path == f"/bags/{name}"
        return (iter([tarball[i:i+256] for i in range(0, len(tarball), 256)]),
                {"size": len(tarball)})
    container = _fake_container_factory(get_archive=_get_archive)
    _patch_docker_sdk(monkeypatch, container=container)

    out = br.auto_pull_and_clean(name)

    assert out["ok"] is True, out
    assert out["host_path"] == f"{tmp_path}/{name}"
    assert out["error"] == ""
    # Files actually landed.
    extracted = tmp_path / name
    assert (extracted / "metadata.yaml").read_bytes() == b"version: 5\nstorage_id: mcap\n"
    assert (extracted / "bag_0.mcap").read_bytes() == b"fake_mcap_data"
    # Cleanup was called once with the right rm command.
    container.exec_run.assert_called_once()
    cmd_arg = container.exec_run.call_args[0][0]
    assert cmd_arg == ["rm", "-rf", f"/bags/{name}"]


def test_auto_pull_disabled_by_env(monkeypatch):
    _env(monkeypatch, IFSSIM_BAG_AUTO_PULL="0")
    fake = _patch_docker_sdk(monkeypatch)

    out = br.auto_pull_and_clean("any_name")

    assert out["ok"] is False
    assert "disabled" in out["error"]
    # SDK never touched when disabled (from_env not called).
    fake.from_env.assert_not_called() if hasattr(fake, "from_env") else None


def test_auto_pull_docker_sdk_unavailable(monkeypatch):
    _env(monkeypatch)
    _patch_docker_sdk(monkeypatch, from_env_raises=Exception("broken socket"))

    out = br.auto_pull_and_clean("any_name")

    assert out["ok"] is False
    assert "docker SDK" in out["error"] or "unavailable" in out["error"]


def test_auto_pull_container_not_found(monkeypatch, tmp_path):
    _env(monkeypatch)
    monkeypatch.setattr(br, "_HOST_BAGS_DIR", str(tmp_path))
    _patch_docker_sdk(monkeypatch, container=None)  # default: containers.get raises

    out = br.auto_pull_and_clean("any_name")

    assert out["ok"] is False
    assert "not found" in out["error"]


def test_auto_pull_get_archive_failure_keeps_volume_copy(monkeypatch, tmp_path):
    _env(monkeypatch)
    monkeypatch.setattr(br, "_HOST_BAGS_DIR", str(tmp_path))
    def _get_archive(path):
        raise Exception("no such path in container")
    container = _fake_container_factory(get_archive=_get_archive)
    _patch_docker_sdk(monkeypatch, container=container)

    out = br.auto_pull_and_clean("trackdrive_x_20260514_191500")

    assert out["ok"] is False
    assert "get_archive" in out["error"]
    # rm should NOT be called — we don't delete on a failed transfer.
    container.exec_run.assert_not_called()


def test_auto_pull_rm_failure_keeps_ok_with_warning(monkeypatch, tmp_path):
    _env(monkeypatch)
    monkeypatch.setattr(br, "_HOST_BAGS_DIR", str(tmp_path))
    name = "trackdrive_x_20260514_191500"
    tarball = _make_tarball_bytes(name, {"x.mcap": b"data"})

    def _get_archive(path):
        return (iter([tarball]), {"size": len(tarball)})
    def _exec_run(cmd, **kw):
        return (1, b"device or resource busy")
    container = _fake_container_factory(get_archive=_get_archive,
                                         exec_run=_exec_run)
    _patch_docker_sdk(monkeypatch, container=container)

    out = br.auto_pull_and_clean(name)

    # Bag landed → overall ok, but warning surfaced.
    assert out["ok"] is True
    assert "cleanup failed" in out["error"]
    assert "orphan" in out["error"]
    # File still extracted correctly.
    assert (tmp_path / name / "x.mcap").read_bytes() == b"data"


def test_auto_pull_rejects_suspicious_bag_name(monkeypatch, tmp_path):
    _env(monkeypatch)
    monkeypatch.setattr(br, "_HOST_BAGS_DIR", str(tmp_path))
    fake_client = _patch_docker_sdk(monkeypatch)

    for bad in ("../etc", "foo/bar", "-rf"):
        out = br.auto_pull_and_clean(bad)
        assert out["ok"] is False, f"should refuse {bad!r}"
        assert "suspicious" in out["error"]
    # No container ever fetched, no get_archive ever called.
    fake_client.containers.get.assert_not_called()


def test_auto_pull_rejects_tarball_with_traversal(monkeypatch, tmp_path):
    """If a malicious bag's tarball contains a member that would
    extract outside /host_bags/, we refuse the extract and return
    an error. Defence in depth on top of the bag-name sanitisation."""
    _env(monkeypatch)
    monkeypatch.setattr(br, "_HOST_BAGS_DIR", str(tmp_path))

    import io as _io
    import tarfile as _tar
    buf = _io.BytesIO()
    with _tar.open(fileobj=buf, mode="w") as tar:
        # Member's name escapes via ..
        info = _tar.TarInfo(name="../sneaky/escape.txt")
        info.size = 4
        tar.addfile(info, fileobj=_io.BytesIO(b"evil"))
    tarball = buf.getvalue()

    def _get_archive(path):
        return (iter([tarball]), {"size": len(tarball)})
    container = _fake_container_factory(get_archive=_get_archive)
    _patch_docker_sdk(monkeypatch, container=container)

    out = br.auto_pull_and_clean("legit_name_20260514")

    assert out["ok"] is False
    assert "suspicious member path" in out["error"]
    # Nothing extracted outside host_dir.
    assert not (tmp_path.parent / "sneaky").exists()


def test_is_auto_pull_enabled_default_is_on(monkeypatch):
    monkeypatch.delenv("IFSSIM_BAG_AUTO_PULL", raising=False)
    assert br.is_auto_pull_enabled() is True


def test_is_auto_pull_enabled_off_values(monkeypatch):
    for off in ("0", "false", "no", "off", "FALSE", "  0 "):
        monkeypatch.setenv("IFSSIM_BAG_AUTO_PULL", off)
        assert br.is_auto_pull_enabled() is False, f"{off!r} should disable"
