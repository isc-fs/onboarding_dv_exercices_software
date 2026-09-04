# IFS Onboarding

Isolated training repo for new ISC Driverless members.

You **do not** clone the full IFSSIM Unreal project for this track.
Download a **pre-built simulator release**, clone **this** repo, work through
the exercises, fill the gaps in `pipeline/`, then drive with Docker + Mission Control.

## What is in here

| Path | Role |
|------|------|
| [`onboarding/`](onboarding/) | Toy exercises + student GUIDE |
| [`pipeline/`](pipeline/) | Full autonomy stack with `STUDENT TODO` gaps |
| [`ros2/`](ros2/) | `ifssim_bridge` + `sim_supervisor` (talks to the sim binary) |
| [`docker/`](docker/) + `docker-compose.yml` | Bridge, pipeline, Mission Control, Lichtblick |
| [`tools/mission_control/`](tools/mission_control/) | Session UI / API |
| [`python/ifssim/`](python/ifssim/) | RPC client used by Mission Control |

No UE5 `Source/`, `Content/`, or editor project — that stays in IFSSIM releases.

## Quick start

1. Follow **[SETUP.md](SETUP.md)** (Docker + release binary + `compose up`).
2. Work through **[onboarding/GUIDE.md](onboarding/GUIDE.md)**.
3. After TODOs are filled, refresh the stack and run a Mission Control session.

## Mentors

See [`onboarding/README.md`](onboarding/README.md). The gapped `pipeline/`
mirrors the `onboarding` branch of
[IFS08-DV-PIPELINE](https://github.com/isc-fs/IFS08-DV-PIPELINE); production
answers live on that repo’s `dev` branch.

---

*ISC Racing Team*
