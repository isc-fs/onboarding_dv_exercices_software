"""Unit tests for the sim uDV-emulator AS state machine.

Pure module — no rclpy. Pins the (current × intent × estop × dv_status)
→ next-AS-state table that must match the C firmware. Imports the
canonical AS/DV bytes from mission_control.interface_contract (single
source), so this also guards that the two packages agree.
"""
from __future__ import annotations

import os
import sys

# sim_supervisor package root (parent of test/).
_PKG_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), os.pardir))
sys.path.insert(0, _PKG_ROOT)

# mission_control package root: repo/pipeline/mission_control. Walk up
# from ros2/src/sim_supervisor/test/ to the repo root, then down.
_REPO_ROOT = os.path.abspath(
    os.path.join(os.path.dirname(__file__), *([os.pardir] * 4)))
sys.path.insert(0, os.path.join(_REPO_ROOT, "pipeline", "mission_control"))

from mission_control.interface_contract import (  # noqa: E402
    AS_OFF,
    AS_EMERGENCY,
    AS_READY,
    AS_DRIVING,
    AS_FINISHED,
    DV_IDLE,
    DV_PREPARING,
    DV_READY,
    DV_RUNNING,
    DV_FINISHED,
    DV_EMERGENCY,
)
from sim_supervisor.as_state_machine import (  # noqa: E402
    OperatorIntent,
    next_as_state,
)


OFF = OperatorIntent.OFF
READY = OperatorIntent.READY
GO = OperatorIntent.GO


# --------------------------------------------------------------------
# Emergency precedence + latching
# --------------------------------------------------------------------

def test_estop_forces_emergency_from_any_state():
    for cur in (AS_OFF, AS_READY, AS_DRIVING, AS_FINISHED):
        assert next_as_state(cur, GO, True, DV_RUNNING, True) == AS_EMERGENCY


def test_pipeline_emergency_forces_emergency():
    assert next_as_state(AS_DRIVING, GO, False, DV_EMERGENCY, True) == \
        AS_EMERGENCY


def test_emergency_latches_until_operator_off():
    # estop released + pipeline cleared, but operator still armed → latch.
    assert next_as_state(AS_EMERGENCY, GO, False, DV_READY, True) == \
        AS_EMERGENCY
    assert next_as_state(AS_EMERGENCY, READY, False, DV_IDLE, True) == \
        AS_EMERGENCY
    # Disarm clears it.
    assert next_as_state(AS_EMERGENCY, OFF, False, DV_IDLE, True) == AS_OFF


# --------------------------------------------------------------------
# Off / arming
# --------------------------------------------------------------------

def test_off_when_disarmed():
    assert next_as_state(AS_READY, OFF, False, DV_READY, True) == AS_OFF


def test_no_runnable_mission_stays_off():
    assert next_as_state(AS_OFF, GO, False, DV_READY, False) == AS_OFF
    assert next_as_state(AS_OFF, READY, False, DV_IDLE, False) == AS_OFF


def test_ready_intent_arms():
    assert next_as_state(AS_OFF, READY, False, DV_IDLE, True) == AS_READY


# --------------------------------------------------------------------
# The go handshake — gated on DV_READY
# --------------------------------------------------------------------

def test_go_holds_at_ready_until_prepared():
    # GO pressed but pipeline still preparing → hold at Ready (triggers
    # mission_control to configure; does NOT drive yet).
    assert next_as_state(AS_OFF, GO, False, DV_IDLE, True) == AS_READY
    assert next_as_state(AS_READY, GO, False, DV_PREPARING, True) == AS_READY


def test_go_drives_once_prepared():
    assert next_as_state(AS_READY, GO, False, DV_READY, True) == AS_DRIVING


def test_driving_persists_while_running():
    assert next_as_state(AS_DRIVING, GO, False, DV_RUNNING, True) == AS_DRIVING


# --------------------------------------------------------------------
# Finish
# --------------------------------------------------------------------

def test_driving_to_finished_on_dv_finished():
    assert next_as_state(AS_DRIVING, GO, False, DV_FINISHED, True) == \
        AS_FINISHED


def test_finished_latches_until_rearm():
    assert next_as_state(AS_FINISHED, GO, False, DV_RUNNING, True) == \
        AS_FINISHED
    # Re-arm (READY) releases the latch.
    assert next_as_state(AS_FINISHED, READY, False, DV_IDLE, True) == AS_READY
    assert next_as_state(AS_FINISHED, OFF, False, DV_IDLE, True) == AS_OFF
