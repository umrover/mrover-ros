import keyboard
import time
import numpy as np
import moteus
import asyncio
import math
from typing import Optional, Literal


class JointDEController:
    def __init__(self) -> None:
        self.DEBUG_MODE_ONLY = False
        self.g_velocity_mode = True
        self.m1_m2_pos_offset = None
        self.POSITION_FOR_VELOCITY_CONTROL = math.nan
        self.MAX_TORQUE = 0.5
        self.ROVER_NODE_TO_MOTEUS_WATCHDOG_TIMEOUT_S = 0.15
        self.MAX_REV_PER_SEC: float = 12
        self.current_pos_state: Literal[""] = ""
        self.time_since_last_changed: float = time.time()

        self.positions_by_key = {
            "i": (0.5, 0),  # pitch 90
            "j": (0, -0.5),  # roll -90
            "k": (-0.5, 0),  # pitch - 90
            "l": (0, 0.5),  # roll 90
        }

        self.sequence_positions = ["i", "k", "j", "l"]

        self.TRANSFORMATION_MATRIX = np.array([[40, 40], [40, -40]])
        self.INVERSE_TRANS_MATRIX = np.linalg.inv(self.TRANSFORMATION_MATRIX)

        self.controller_1 = moteus.Controller(id=0x20)
        self.controller_2 = moteus.Controller(id=0x21)

        self.g_controller_1_rev = None
        self.g_controller_2_rev = None

        if self.DEBUG_MODE_ONLY:
            self.g_controller_1_rev = 0
            self.g_controller_2_rev = 0

        self.g_prev_pitch, self.g_prev_roll = math.nan, math.nan
        self.g_prev_pitch_pos, self.g_prev_roll_pos = math.nan, math.nan

    async def fix_controller_if_error_and_return_pos(self, controller) -> Optional[float]:
        state = await controller.query()
        controller_not_found = self.moteus_not_found(state)

        if controller_not_found:
            print("CONTROLLER NOT FOUND")
            await controller.set_stop()
            return None
        else:
            return state.values[moteus.Register.POSITION]

    def moteus_not_found(self, state) -> bool:
        return (
            state is None
            or not hasattr(state, "values")
            or moteus.Register.FAULT not in state.values
            or moteus.Register.MODE not in state.values
            or state.values[moteus.Register.MODE] == 11  # timeout
        )

    def transform_coordinates_and_clamp(self, pitch, roll) -> tuple:
        # Define the transformation matrix

        # Create the input vector
        input_vector = np.array([pitch, roll])

        # Perform matrix multiplication
        result_vector = np.dot(self.TRANSFORMATION_MATRIX, input_vector)

        # Extract y1 and y2 from the result vector
        m1, m2 = result_vector

        # if abs(m1) > self.MAX_REV_PER_SEC or abs(m2) > self.MAX_REV_PER_SEC:
        #     larger_value = max(abs(m1), abs(m2))
        #     m1 = (m1 / larger_value) * self.MAX_REV_PER_SEC
        #     m2 = (m2 / larger_value) * self.MAX_REV_PER_SEC

        return m1, m2

    def adjust_pitch_roll(self, believed_pitch, believed_roll) -> None:
        print(f"Adjusting pitch and roll to be {believed_pitch} and {believed_roll}")

        if self.g_controller_1_rev is None or self.g_controller_2_rev is None:
            print("Failure to adjust because no current moteus data")
            return

        moteus_believed_pos = np.array([self.g_controller_1_rev, self.g_controller_2_rev])
        user_believed_pitch_roll = np.array([believed_pitch, believed_roll])

        user_believed_pos = np.dot(self.TRANSFORMATION_MATRIX, user_believed_pitch_roll)

        self.m1_m2_pos_offset = moteus_believed_pos - user_believed_pos

    async def move_position(self, pitch, roll) -> None:
        user_desired_pitch_roll = np.array([pitch, roll])

        user_desired_position_vector = np.dot(self.TRANSFORMATION_MATRIX, user_desired_pitch_roll)
        motor_position_vector = user_desired_position_vector + self.m1_m2_pos_offset
        m1pos, m2pos = motor_position_vector

        if self.g_prev_pitch_pos != pitch or self.g_prev_roll_pos != roll:
            print(f"pitch: {pitch}, roll: {roll}, m1pos: {m1pos}, m2pos: {m2pos}")
            self.g_prev_pitch_pos = pitch
            self.g_prev_roll_pos = roll

        if self.DEBUG_MODE_ONLY:
            return

        await self.controller_1.set_position(
            position=m1pos,
            velocity=self.MAX_REV_PER_SEC / 4,
            velocity_limit=self.MAX_REV_PER_SEC,
            maximum_torque=self.MAX_TORQUE,
            watchdog_timeout=self.ROVER_NODE_TO_MOTEUS_WATCHDOG_TIMEOUT_S,
            query=True,
        )
        await self.controller_2.set_position(
            position=m2pos,
            velocity=self.MAX_REV_PER_SEC / 4,
            velocity_limit=self.MAX_REV_PER_SEC,
            maximum_torque=self.MAX_TORQUE,
            watchdog_timeout=self.ROVER_NODE_TO_MOTEUS_WATCHDOG_TIMEOUT_S,
            query=True,
        )

        return_pos_1 = await self.fix_controller_if_error_and_return_pos(self.controller_1)
        if return_pos_1 is not None:
            self.g_controller_1_rev = return_pos_1
        return_pos_2 = await self.fix_controller_if_error_and_return_pos(self.controller_2)
        if return_pos_2 is not None:
            self.g_controller_2_rev = return_pos_2

    async def run_position_state_machine(self) -> None:
        if time.time() - self.time_since_last_changed > 5:
            current_index = self.sequence_positions.index(self.current_pos_state)
            next_index = (current_index + 1) % len(self.sequence_positions)
            next_pos_state = self.sequence_positions[next_index]
            print(f"Changing position!")
            self.current_pos_state = next_pos_state
            self.time_since_last_changed = time.time()

        pitch, roll = self.positions_by_key[self.current_pos_state]
        await self.move_position(pitch, roll)


class MainLoop:
    def __init__(self, joint_de_controller) -> None:
        self.joint_de_controller = joint_de_controller

    async def move_velocity(self, pitch, roll) -> None:
        m1rps, m2rps = self.joint_de_controller.transform_coordinates_and_clamp(pitch, roll)

        if self.joint_de_controller.g_prev_pitch != pitch or self.joint_de_controller.g_prev_roll != roll:
            print(f"pitch: {pitch}, roll: {roll}, m1rps: {m1rps}, m2rps: {m2rps}")
            motorsrps = np.array([m1rps, m2rps])
            expected_pitch, expected_roll = np.dot(self.joint_de_controller.INVERSE_TRANS_MATRIX, motorsrps)
            print(f"expected_pitch: {expected_pitch} expected_roll: {expected_roll}")
            self.joint_de_controller.g_prev_pitch = pitch
            self.joint_de_controller.g_prev_roll = roll

        if self.joint_de_controller.DEBUG_MODE_ONLY:
            return

        if abs(m1rps) > 1e-5:
            await self.joint_de_controller.controller_1.set_position(
                position=self.joint_de_controller.POSITION_FOR_VELOCITY_CONTROL,
                velocity=m1rps,
                velocity_limit=self.joint_de_controller.MAX_REV_PER_SEC,
                maximum_torque=self.joint_de_controller.MAX_TORQUE,
                watchdog_timeout=self.joint_de_controller.ROVER_NODE_TO_MOTEUS_WATCHDOG_TIMEOUT_S,
                query=True,
            )
        else:
            await self.joint_de_controller.controller_1.set_brake()
        if abs(m2rps) > 1e-5:
            await self.joint_de_controller.controller_2.set_position(
                position=self.joint_de_controller.POSITION_FOR_VELOCITY_CONTROL,
                velocity=m2rps,
                velocity_limit=self.joint_de_controller.MAX_REV_PER_SEC,
                maximum_torque=self.joint_de_controller.MAX_TORQUE,
                watchdog_timeout=self.joint_de_controller.ROVER_NODE_TO_MOTEUS_WATCHDOG_TIMEOUT_S,
                query=True,
            )
        else:
            await self.joint_de_controller.controller_2.set_brake()

        return_pos_1 = await self.joint_de_controller.fix_controller_if_error_and_return_pos(
            self.joint_de_controller.controller_1
        )
        if return_pos_1 is not None:
            self.joint_de_controller.g_controller_1_rev = return_pos_1
        return_pos_2 = await self.joint_de_controller.fix_controller_if_error_and_return_pos(
            self.joint_de_controller.controller_2
        )
        if return_pos_2 is not None:
            self.joint_de_controller.g_controller_2_rev = return_pos_2

    async def main(self) -> None:
        mapping_by_key = {
            "w": (8 / 60 * 0.3, 0),
            "a": (0, -8 / 60 * 0.3),
            "s": (-8 / 60 * 0.3, 0),
            "d": (0, 8 / 60 * 0.3),
        }

        print("Controls are the following:")
        print("X to exit.")
        print("WASD to control in velocity control")
        print("Calibrate with I/J/K/L to access position control")
        print("P to go to looping position control. V to go to velocity control")

        while True:
            if keyboard.is_pressed("x"):
                await self.move_velocity(0, 0)
                print("EXITING PROGRAM!")
                return

            current_adjust_state = {key: keyboard.is_pressed(key) for key in ["i", "j", "k", "l"]}
            for key in self.joint_de_controller.positions_by_key:
                if current_adjust_state[key]:
                    self.joint_de_controller.current_pos_state = key
                    believed_pitch = self.joint_de_controller.positions_by_key[key][0]
                    believed_roll = self.joint_de_controller.positions_by_key[key][1]
                    self.joint_de_controller.adjust_pitch_roll(believed_pitch, believed_roll)

                    self.joint_de_controller.g_velocity_mode = True
                    print("Entering Velocity Mode as a safety after adjusting")
                    break

            if self.joint_de_controller.g_velocity_mode:
                if keyboard.is_pressed("p"):
                    if self.joint_de_controller.m1_m2_pos_offset is None:
                        print("Failure to go into Looping Position mode. Please calibrate with i/j/k/l first.")
                    else:
                        print("Entering the Looping Position Mode")
                        self.joint_de_controller.g_velocity_mode = False
                        continue
                current_state = {key: keyboard.is_pressed(key) for key in ["w", "a", "s", "d"]}

                pitch, roll = (0, 0)
                for key in mapping_by_key:
                    if current_state[key]:
                        pitch_change, roll_change = mapping_by_key[key]
                        pitch += pitch_change
                        roll += roll_change
                await self.move_velocity(pitch, roll)
            else:
                if keyboard.is_pressed("V"):
                    print("Entering Velocity Mode")
                    self.joint_de_controller.g_velocity_mode = True
                    continue
                await self.joint_de_controller.run_position_state_machine()

            time.sleep(0.1)


if __name__ == "__main__":
    joint_de_controller = JointDEController()
    main_loop = MainLoop(joint_de_controller)
    asyncio.run(main_loop.main())
