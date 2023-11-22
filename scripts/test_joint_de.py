import keyboard
import time
import numpy as np
import moteus
import asyncio
import math

DEBUG_MODE_ONLY = True

controller_1 = moteus.Controller(id=0x20)
controller_2 = moteus.Controller(id=0x21)


def print_key(key):
    print(f"Key pressed: {key}")


MAX_REV_PER_SEC = 8 / 60  # 8 RPM to RPS


def transform_coordinates_and_clamp(x1, x2):
    # Define the transformation matrix
    transformation_matrix = np.array([[40, 40], [40, -40]])

    # Create the input vector
    input_vector = np.array([x1, x2])

    # Perform matrix multiplication
    result_vector = np.dot(transformation_matrix, input_vector)

    # Extract y1 and y2 from the result vector
    y1, y2 = result_vector

    if abs(y1) > MAX_REV_PER_SEC or abs(y2) > MAX_REV_PER_SEC:
        larger_value = max(abs(y1), abs(y2))
        y1 = (y1 / larger_value) * MAX_REV_PER_SEC
        y2 = (y2 / larger_value) * MAX_REV_PER_SEC

    return y1, y2


POSITION_FOR_VELOCITY_CONTROL = math.nan
MAX_TORQUE = 0.3
ROVER_NODE_TO_MOTEUS_WATCHDOG_TIMEOUT_S = 0.15


async def move_velocity(pitch, roll):
    m1rps, m2rps = transform_coordinates_and_clamp(pitch, roll)
    print(f"pitch: {pitch}, roll: {roll}, m1rps: {m1rps}, m2rps: {m2rps}")

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

    async def fix_controller_if_error(controller):
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

    await fix_controller_if_error(controller_1)
    await fix_controller_if_error(controller_2)


async def main():
    mapping_by_key = {
        # pitch, roll
        "w": (1, 0),
        "a": (0, -1),
        "s": (-1, 0),
        "d": (0, 1),
    }

    while True:
        current_state = {key: keyboard.is_pressed(key) for key in ["w", "a", "s", "d"]}

        pitch, roll = (0, 0)
        for key in mapping_by_key:
            if current_state[key]:
                pitch_change, roll_change = mapping_by_key[key]
                pitch += pitch_change
                roll += roll_change
        await move_velocity(pitch, roll)

        # Add a delay to reduce CPU usage
        time.sleep(0.1)


if __name__ == "__main__":
    asyncio.run(main())
