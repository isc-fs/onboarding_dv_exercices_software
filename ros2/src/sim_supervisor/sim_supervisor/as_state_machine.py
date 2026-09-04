"""Pure AS state-machine for the sim uDV emulator.

On the real car the uDV owns the AS (Autonomous System) state machine and
publishes /assi/state; the AMI board + RES buttons are its inputs. In the
sim there is no uDV, so sim_supervisor emulates it: this module is the
pure transition core (no rclpy), so the emulator and the C firmware are
provably the *same* machine — testing the handshake in sim validates the
car. The node owns the ROS plumbing (publishing /assi/state + /ami/mission,
relaying /ctrl/cmd, serving /force_ebs).

Inputs:
  * operator intent (OFF / READY / GO) — the sim's stand-in for the
    AMI "armed" state + the RES go button, driven by the backend/CLI.
  * estop — the sim's RES emergency-stop.
  * dv_status — mission_control's /dv/status byte (the prepare/run
    handshake): "go" (Ready→Driving) is gated on DV_READY, so the car
    never drives before autonomy is genuinely prepared, exactly as the
    firmware gates it.
  * mission_runnable — whether a runnable mission is selected.

The transition is a pure function of (current AS state, those inputs).
"""
from __future__ import annotations

from enum import IntEnum

from mission_control.interface_contract import (
    AS_DRIVING,
    AS_EMERGENCY,
    AS_FINISHED,
    AS_OFF,
    AS_READY,
    DV_EMERGENCY,
    DV_FINISHED,
    DV_READY,
)


class OperatorIntent(IntEnum):
    """Sim operator panel intent — the AMI/RES stand-in (wire: /sim/intent)."""

    OFF = 0      # disarmed
    READY = 1    # armed / prepare (RES go not pressed yet)
    GO = 2       # RES go pressed — run the mission


def next_as_state(
    current: int,
    intent: int,
    estop: bool,
    dv_status: int,
    mission_runnable: bool,
) -> int:
    """Return the next AS-state byte.

    Precedence (highest first):
      1. estop or pipeline-raised emergency (DV_EMERGENCY) → AS_EMERGENCY.
      2. Emergency latches until the operator disarms (intent OFF).
      3. intent OFF, or no runnable mission → AS_OFF.
      4. intent READY → AS_READY (triggers mission_control to configure).
      5. intent GO → AS_DRIVING, but ONLY once dv_status == DV_READY
         (the prepared handshake); otherwise hold at AS_READY. While
         driving, DV_FINISHED → AS_FINISHED (latched until re-armed).
    """
    intent = int(intent)
    cur = int(current)
    dv = int(dv_status)

    if estop or dv == DV_EMERGENCY:
        return AS_EMERGENCY
    if intent == OperatorIntent.OFF:
        return AS_OFF
    if cur == AS_EMERGENCY:
        # Latched: only intent OFF (handled above) clears an emergency.
        return AS_EMERGENCY
    if not mission_runnable:
        # Can't arm or go without a runnable mission selected.
        return AS_OFF
    if intent == OperatorIntent.READY:
        return AS_READY

    # intent == GO
    if cur == AS_DRIVING:
        return AS_FINISHED if dv == DV_FINISHED else AS_DRIVING
    if cur == AS_FINISHED:
        # Latch finished until the operator drops to READY (re-arm) or OFF.
        return AS_FINISHED
    # Not yet driving — gate the go on the pipeline being prepared.
    return AS_DRIVING if dv == DV_READY else AS_READY
