import keyboard
import time
import numpy as np
import moteus
import asyncio
import math

DEBUG_MODE_ONLY = True
g_velocity_mode = True  # As opposed to looping position control mode
pitch_roll_offset = None

POSITION_FOR_VELOCITY_CONTROL = math.nan
MAX_TORQUE = 0.3
ROVER_NODE_TO_MOTEUS_WATCHDOG_TIMEOUT_S = 0.15

g_prev_pitch, g_prev_roll = math.nan, math.nan
g_prev_pitch_pos, g_prev_roll_pos = math.nan, math.nan

TRANSFORMATION_MATRIX = np.array([[40, 40], [40, -40]])  # m1/m2 = TRANSFORMATION_MATRIX * pitch/roll
INVERSE_TRANS_MATRIX = np.linalg.inv(TRANSFORMATION_MATRIX)  # pitch/roll = INVERSE_TRANS_MATRIX * m1/m2

controller_1 = moteus.Controller(id=0x20)
controller_2 = moteus.Controller(id=0x21)

g_controller_1_rev = None
g_controller_2_rev = None

if DEBUG_MODE_ONLY:
    g_controller_1_rev = 0
    g_controller_2_rev = 0

MAX_REV_PER_SEC = 8 / 60  # 8 RPM to RPS

current_pos_state = ""
time_since_last_changed = time.time()

positions_by_key = {
    "i": (0.5, 0),  # pitch 90
    "j": (0, -0.5),  # roll -90
    "k": (-0.5, 0),  # pitch - 90
    "l": (0, 0.5),  # roll 90
}


async def fix_controller_if_error_and_return_pos(controller, pos_val_to_update):
    global g_controller_1_rev, g_controller_2_rev

    def moteus_not_found(state):
        return (
            state is None
            or not hasattr(state, "values")
            or moteus.Register.FAULT not in state.values
            or moteus.Register.MODE not in state.values
        )

    state = await controller.query()
    controller_not_found = moteus_not_found(state)
    if controller_not_found:
        await controller.set_stop()
        return None
    else:
        return state.values[moteus.Register.POSITION]


def adjust_pitch_roll(believed_pitch, believed_roll):
    global pitch_roll_offset
    print(f"Adjusting pitch and roll to be {believed_pitch} and {believed_roll}")

    # TODO - get what the moteus thinks its revolution is
    if g_controller_1_rev is None or g_controller_2_rev is None:
        print("Failure to adjust because no current moteus data")
        return

    moteus_believed_rev = np.array([g_controller_1_rev, g_controller_2_rev])
    moteus_believed_pitch_roll = np.dot(INVERSE_TRANS_MATRIX, moteus_believed_rev)

    user_believed_pitch_roll = np.array([believed_pitch, believed_roll])

    pitch_roll_offset = moteus_believed_pitch_roll - user_believed_pitch_roll


async def move_position(pitch, roll):
    global g_controller_1_rev, g_controller_2_rev
    global g_prev_pitch_pos, g_prev_roll_pos

    user_desired_pitch_roll = np.array([pitch, roll])
    moteus_desired_pitch_roll = pitch_roll_offset + user_desired_pitch_roll

    # Perform matrix multiplication
    motor_position_vector = np.dot(TRANSFORMATION_MATRIX, moteus_desired_pitch_roll)

    # Extract y1 and y2 from the result vector
    m1pos, m2pos = motor_position_vector

    if g_prev_pitch_pos != pitch or g_prev_roll_pos != roll:
        print(f"pitch: {pitch}, roll: {roll}, m1pos: {m1pos}, m2pos: {m2pos}")
        g_prev_pitch_pos = pitch
        g_prev_roll_pos = roll

    if DEBUG_MODE_ONLY:
        return

    await controller_1.set_position(
        position=m1pos,
        velocity=MAX_REV_PER_SEC / 4,
        velocity_limit=MAX_REV_PER_SEC,
        maximum_torque=MAX_TORQUE,
        watchdog_timeout=ROVER_NODE_TO_MOTEUS_WATCHDOG_TIMEOUT_S,
        query=True,
    )
    await controller_2.set_position(
        position=m2pos,
        velocity=MAX_REV_PER_SEC / 4,
        velocity_limit=MAX_REV_PER_SEC,
        maximum_torque=MAX_TORQUE,
        watchdog_timeout=ROVER_NODE_TO_MOTEUS_WATCHDOG_TIMEOUT_S,
        query=True,
    )

    return_pos_1 = await fix_controller_if_error_and_return_pos(controller_1)
    if return_pos_1 is not None:
        g_controller_1_rev = return_pos_1
    return_pos_2 = await fix_controller_if_error_and_return_pos(controller_2)
    if return_pos_2 is not None:
        g_controller_2_rev = return_pos_2


sequence_positions = ["i", "k", "j", "l"]


async def run_position_state_machine():
    global current_pos_state, time_since_last_changed
    if time.time() - time_since_last_changed > 5:
        current_index = sequence_positions.index(current_pos_state)
        next_index = (current_index + 1) % len(sequence_positions)
        next_pos_state = sequence_positions[next_index]
        print(f"Changing position!")
        current_pos_state = next_pos_state
        time_since_last_changed = time.time()

    pitch, roll = positions_by_key[current_pos_state]
    await move_position(pitch, roll)


def print_key(key):
    print(f"Key pressed: {key}")


def transform_coordinates_and_clamp(pitch, roll):
    # Define the transformation matrix

    # Create the input vector
    input_vector = np.array([pitch, roll])

    # Perform matrix multiplication
    result_vector = np.dot(TRANSFORMATION_MATRIX, input_vector)

    # Extract y1 and y2 from the result vector
    m1, m2 = result_vector

    if abs(m1) > MAX_REV_PER_SEC or abs(m2) > MAX_REV_PER_SEC:
        larger_value = max(abs(m1), abs(m2))
        m1 = (m1 / larger_value) * MAX_REV_PER_SEC
        m2 = (m2 / larger_value) * MAX_REV_PER_SEC

    return m1, m2


async def move_velocity(pitch, roll):
    global g_prev_pitch, g_prev_roll
    global g_controller_1_rev, g_controller_2_rev
    m1rps, m2rps = transform_coordinates_and_clamp(pitch, roll)
    if g_prev_pitch != pitch or g_prev_roll != roll:
        print(f"pitch: {pitch}, roll: {roll}, m1rps: {m1rps}, m2rps: {m2rps}")
        g_prev_pitch = pitch
        g_prev_roll = roll

    if DEBUG_MODE_ONLY:
        return

    await controller_1.set_position(
        position=POSITION_FOR_VELOCITY_CONTROL,
        velocity=m1rps,
        velocity_limit=MAX_REV_PER_SEC,
        maximum_torque=MAX_TORQUE,
        watchdog_timeout=ROVER_NODE_TO_MOTEUS_WATCHDOG_TIMEOUT_S,
        query=True,
    )
    await controller_2.set_position(
        position=POSITION_FOR_VELOCITY_CONTROL,
        velocity=m2rps,
        velocity_limit=MAX_REV_PER_SEC,
        maximum_torque=MAX_TORQUE,
        watchdog_timeout=ROVER_NODE_TO_MOTEUS_WATCHDOG_TIMEOUT_S,
        query=True,
    )

    return_pos_1 = await fix_controller_if_error_and_return_pos(controller_1)
    if return_pos_1 is not None:
        g_controller_1_rev = return_pos_1
    return_pos_2 = await fix_controller_if_error_and_return_pos(controller_2)
    if return_pos_2 is not None:
        g_controller_2_rev = return_pos_2


async def main():
    global g_velocity_mode
    mapping_by_key = {
        # pitch, roll
        "w": (1, 0),
        "a": (0, -1),
        "s": (-1, 0),
        "d": (0, 1),
    }

    print("Controls are the following:")
    print("X to exit.")
    print("WASD to control in velocity control")
    print("Calibrate with I/J/K/L to access position control")
    print("P to go to looping position control. V to go to velocity control")

    while True:
        if keyboard.is_pressed("x"):
            await move_velocity(0, 0)
            print("EXITING PROGRAM!")
            return

        current_adjust_state = {key: keyboard.is_pressed(key) for key in ["i", "j", "k", "l"]}
        for key in positions_by_key:
            if current_adjust_state[key]:
                global current_pos_state
                current_pos_state = key
                believed_pitch = positions_by_key[key][0]
                believed_roll = positions_by_key[key][1]
                adjust_pitch_roll(believed_pitch, believed_roll)

                g_velocity_mode = True
                print("Entering Velocity Mode as a safety after adjusting")
                break

        if g_velocity_mode:
            if keyboard.is_pressed("P"):
                if pitch_roll_offset is None:
                    print("Failure to go into Looping Position mode. Please calibrate with i/j/k/l first.")
                else:
                    print("Entering the Looping Position Mode")
                    g_velocity_mode = False
                    continue
            current_state = {key: keyboard.is_pressed(key) for key in ["w", "a", "s", "d"]}

            pitch, roll = (0, 0)
            for key in mapping_by_key:
                if current_state[key]:
                    pitch_change, roll_change = mapping_by_key[key]
                    pitch += pitch_change
                    roll += roll_change
            await move_velocity(pitch, roll)
        else:
            if keyboard.is_pressed("V"):
                print("Entering Velocity Mode")
                g_velocity_mode = True
                continue
            await run_position_state_machine()

        # Add a delay to reduce CPU usage
        time.sleep(0.1)


if __name__ == "__main__":
    asyncio.run(main())
