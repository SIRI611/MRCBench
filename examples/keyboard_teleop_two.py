#!/usr/bin/env python3
from __future__ import annotations

from time import sleep
from typing import Iterable
from pynput import keyboard
from pprint import pprint

import click

# --- Stretch Mujoco SDK imports
from stretch_mujoco import StretchMujocoSimulator
from stretch_mujoco.enums.actuators import Actuators
from stretch_mujoco.enums.stretch_cameras import StretchCameras
from stretch_mujoco.enums.stretch_sensors import StretchSensors
# We use CommandMove directly to send name-based moves for robot 2
from stretch_mujoco.datamodels.status_command import CommandMove

# ============ Controls / UI ============
ACTIVE = "r1"  # 'r1', 'r2', or 'both'

def print_keyboard_options():
    click.secho("\n       Keyboard Controls (Two Robots):", fg="yellow")
    click.secho("================================================", fg="yellow")
    print("Target: [1]=Robot1   [2]=Robot2   [0]=Both")
    print("W / A / S / D: Move BASE   (r1 uses built-in base cmd; r2 uses wheel vels)")
    print("T / F / G / H: Move HEAD (tilt/pan)")
    print("I / J / K / L: Move LIFT & ARM")
    print("O / P: Move WRIST YAW")
    print("C / V: Move WRIST PITCH")
    print("E / R: Move WRIST ROLL")
    print("N / M: Open & Close GRIPPER")
    print("Z : Print status (Robot1)")
    print("Q : Stop")
    click.secho("================================================\n", fg="yellow")

# ============ Low-level helpers for name-based commands (r2) ============
def _move_by_name(sim: StretchMujocoSimulator, actuator_name: str, delta: float) -> None:
    """Send a relative move directly by actuator name (works for pos/vel types)."""
    with sim._command_lock:  # noqa: SLF001 – OK inside same process
        cmd = sim.data_proxies.get_command()
        cmd.set_move_by(CommandMove(actuator_name=actuator_name, pos=delta, trigger=True))
        sim.data_proxies.set_command(cmd)

# We will simulate "set velocity" for wheel actuators by maintaining the last ctrl we sent,
# then issuing a relative delta (move_by) to reach the new value.
_r2_last_ctrl = {"r2_left_wheel_vel": 0.0, "r2_right_wheel_vel": 0.0}

def _set_vel_by_name(sim: StretchMujocoSimulator, actuator_name: str, value: float) -> None:
    last = _r2_last_ctrl.get(actuator_name, 0.0)
    delta = float(value) - float(last)
    if abs(delta) > 1e-6:
        _move_by_name(sim, actuator_name, delta)
        _r2_last_ctrl[actuator_name] = float(value)

def _stop_r2_base(sim: StretchMujocoSimulator) -> None:
    _set_vel_by_name(sim, "r2_left_wheel_vel", 0.0)
    _set_vel_by_name(sim, "r2_right_wheel_vel", 0.0)

# ============ Key mapping for joints ============
def _for_targets(sim: StretchMujocoSimulator, f_r1, f_r2):
    global ACTIVE
    if ACTIVE in ("r1", "both"):
        f_r1(sim)
    if ACTIVE in ("r2", "both"):
        f_r2(sim)

def _move_joint(sim: StretchMujocoSimulator, r1_name: Actuators, r2_name: str, delta: float):
    """Move a named joint on both robots (r1 via enum, r2 via actuator name)."""
    _for_targets(
        sim,
        lambda s: s.move_by(r1_name, delta),
        lambda s: _move_by_name(s, r2_name, delta),
    )

# ============ High-level teleop handlers ============
# Tunables for base motion (r2 wheels are unitless ctrl; keep conservative)
R1_BASE_STEP = 27
R1_ROT_STEP  = 20
R2_FWD_SPEED = 3.0   # try in [-6, 6]; clamp below
R2_TURN_SPEED = 3.0  # try in [-6, 6]

def _clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, x))

def _drive_r2(sim: StretchMujocoSimulator, v: float, omega: float) -> None:
    """
    Differential-drive style: map v, omega to wheel velocities.
    We use a simple mapping (left=v-omega, right=v+omega) and clamp to actuator range.
    """
    left  = _clamp(v - omega, -6.0, 6.0)
    right = _clamp(v + omega, -6.0, 6.0)
    _set_vel_by_name(sim, "r2_left_wheel_vel", left)
    _set_vel_by_name(sim, "r2_right_wheel_vel", right)

def keyboard_control(key: str | None, sim: StretchMujocoSimulator, keys_down: Iterable[str]):
    global ACTIVE

    # Target toggles
    if key == "1":
        ACTIVE = "r1"
        click.secho("Active target: Robot 1", fg="cyan")
        return
    elif key == "2":
        ACTIVE = "r2"
        click.secho("Active target: Robot 2", fg="cyan")
        return
    elif key == "0":
        ACTIVE = "both"
        click.secho("Active target: BOTH robots", fg="cyan")
        return

    # --- BASE ---
    if key in ("w", "s", "a", "d"):
        # Robot 1 base uses built-in pseudo actuators
        if ACTIVE in ("r1", "both"):
            if key == "w":
                sim.move_by(Actuators.base_translate,  R1_BASE_STEP)
            elif key == "s":
                sim.move_by(Actuators.base_translate, -R1_BASE_STEP)
            elif key == "a":
                sim.move_by(Actuators.base_rotate,  R1_ROT_STEP)
            elif key == "d":
                sim.move_by(Actuators.base_rotate, -R1_ROT_STEP)

        # Robot 2 base uses velocity actuators (name-based)
        if ACTIVE in ("r2", "both"):
            v = 0.0
            omega = 0.0
            # Use the full set of keys currently down to compute v, omega,
            # so W+A diagonals behave nicely.
            if "w" in keys_down:
                v += R2_FWD_SPEED
            if "s" in keys_down:
                v -= R2_FWD_SPEED
            if "a" in keys_down:
                omega += R2_TURN_SPEED
            if "d" in keys_down:
                omega -= R2_TURN_SPEED
            _drive_r2(sim, v, omega)
        return

    # --- HEAD ---
    if key == "t":  # tilt up
        _move_joint(sim, Actuators.head_tilt, "r2_head_tilt", +0.2)
    elif key == "g":  # tilt down
        _move_joint(sim, Actuators.head_tilt, "r2_head_tilt", -0.2)
    elif key == "f":  # pan left
        _move_joint(sim, Actuators.head_pan, "r2_head_pan", +0.2)
    elif key == "h":  # pan right
        _move_joint(sim, Actuators.head_pan, "r2_head_pan", -0.2)

    # --- ARM / LIFT ---
    elif key == "i":  # lift up
        _move_joint(sim, Actuators.lift, "r2_lift", +0.1)
    elif key == "k":  # lift down
        _move_joint(sim, Actuators.lift, "r2_lift", -0.1)
    elif key == "j":  # arm retract
        _move_joint(sim, Actuators.arm, "r2_arm", -0.05)
    elif key == "l":  # arm extend
        _move_joint(sim, Actuators.arm, "r2_arm", +0.05)

    # --- WRIST ---
    elif key == "o":  # yaw +
        _move_joint(sim, Actuators.wrist_yaw, "r2_wrist_yaw", +0.2)
    elif key == "p":  # yaw -
        _move_joint(sim, Actuators.wrist_yaw, "r2_wrist_yaw", -0.2)
    elif key == "c":  # pitch +
        _move_joint(sim, Actuators.wrist_pitch, "r2_wrist_pitch", +0.2)
    elif key == "v":  # pitch -
        _move_joint(sim, Actuators.wrist_pitch, "r2_wrist_pitch", -0.2)
    elif key == "e":  # roll +
        _move_joint(sim, Actuators.wrist_roll, "r2_wrist_roll", +0.2)
    elif key == "r":  # roll -
        _move_joint(sim, Actuators.wrist_roll, "r2_wrist_roll", -0.2)

    # --- GRIPPER ---
    elif key == "n":  # open
        _move_joint(sim, Actuators.gripper, "r2_gripper", +0.07)
    elif key == "m":  # close
        _move_joint(sim, Actuators.gripper, "r2_gripper", -0.07)

    # --- OTHER ---
    elif key == "z":
        # (Robot 1 status — the SDK’s Status mirrors r1’s joints)
        pprint(sim.pull_status())
    elif key == "q":
        sim.stop()

def keyboard_control_release(key: str | None, sim: StretchMujocoSimulator):
    # Stop continuous base motion on release for both robots.
    if key in ("w", "s", "a", "d"):
        if ACTIVE in ("r1", "both"):
            sim.set_base_velocity(0, 0)
        if ACTIVE in ("r2", "both"):
            _stop_r2_base(sim)

# ============ Multi-key support ============
key_buffer: list[keyboard.Key] = []

def on_press(key):
    global key_buffer
    if key not in key_buffer and len(key_buffer) < 4:
        key_buffer.append(key)

def on_release(key, sim: StretchMujocoSimulator):
    global key_buffer
    if key in key_buffer:
        key_buffer.remove(key)
    if isinstance(key, keyboard.KeyCode):
        keyboard_control_release(key.char, sim)

# ============ CLI / Main loop ============
@click.command()
@click.option("--scene-xml-path", type=str, default=None, help="Path to the scene xml file")
@click.option("--imagery-nav", is_flag=True, help="Show only the Navigation camera")
@click.option("--imagery", is_flag=True, help="Show all the cameras' imagery")
@click.option("--lidar", is_flag=True, help="Show the lidar scan in Matplotlib")
@click.option("--print-ratio", is_flag=True, help="Print sim-to-real time ratio")
def main(scene_xml_path: str | None, imagery_nav: bool, imagery: bool, lidar: bool, print_ratio: bool):
    cameras_to_use = StretchCameras.all() if imagery else []
    if imagery_nav:
        cameras_to_use = [StretchCameras.cam_nav_rgb]
        imagery = True
    use_imagery = imagery or imagery_nav

    sim = StretchMujocoSimulator(
        scene_xml_path=scene_xml_path,
        cameras_to_use=cameras_to_use,
    )

    try:
        sim.start()
        sim.move_to(Actuators.lift, 0.50)
        sim.move_to(Actuators.arm, 0.15)
        sim.wait_until_at_setpoint(Actuators.lift)
        sim.wait_until_at_setpoint(Actuators.arm)
        print_keyboard_options()
        click.secho("Tip: press 1 / 2 / 0 to choose Robot1 / Robot2 / BOTH.\n", fg="green")

        listener = keyboard.Listener(on_press=on_press, on_release=lambda key: on_release(key, sim))
        listener.start()

        while sim.is_running():
            # Gather currently pressed character keys
            keys_down_chars = [k.char for k in key_buffer if isinstance(k, keyboard.KeyCode) and k.char]
            # Handle each key once per cycle
            for key in keys_down_chars:
                keyboard_control(key, sim, keys_down_chars)

            if not lidar and not use_imagery:
                sleep(0.05)

            if print_ratio:
                try:
                    print(sim.pull_status().sim_to_real_time_ratio_msg)
                except:  # noqa: E722
                    ...

            if use_imagery:
                try:
                    from examples.camera_feeds import show_camera_feeds_sync
                    show_camera_feeds_sync(sim, False)
                except:  # noqa: E722
                    ...

            if lidar:
                try:
                    from examples.laser_scan import show_laser_scan
                    sensor_data = sim.pull_sensor_data()
                    show_laser_scan(scan_data=sensor_data.get_data(StretchSensors.base_lidar))
                except:  # noqa: E722
                    ...

        listener.stop()

    except KeyboardInterrupt:
        sim.stop()


if __name__ == "__main__":
    main()
