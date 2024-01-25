from __future__ import annotations

import threading
import time
from typing import Callable

import rospy
from mrover.msg import StateMachineStructure, StateMachineTransition, StateMachineStateUpdate

from util.state_lib.state_machine import StateMachine


class StatePublisher:
    structure_publisher: rospy.Publisher
    state_publisher: rospy.Publisher
    state_machine: StateMachine
    __structure_thread: threading.Thread
    __state_thread: threading.Thread
    __stop_event: threading.Event

    def __init__(
        self,
        state_machine: StateMachine,
        structure_pub_topic: str,
        structure_update_rate_hz: float,
        state_pub_topic: str,
        state_update_rate_hz: float,
    ):
        self.state_machine = state_machine
        self.structure_publisher = rospy.Publisher(structure_pub_topic, StateMachineStructure, queue_size=1)
        self.state_publisher = rospy.Publisher(state_pub_topic, StateMachineStateUpdate, queue_size=1)
        self.__stop_event = threading.Event()
        self.__structure_thread = threading.Thread(
            target=self.run_at_interval, args=(self.publish_structure, structure_update_rate_hz)
        )
        self.__state_thread = threading.Thread(
            target=self.run_at_interval, args=(self.publish_state, state_update_rate_hz)
        )
        self.__stop = False
        self.__structure_thread.start()
        self.__state_thread.start()

    def stop(self) -> None:
        self.__stop_event.set()

    def publish_structure(self) -> None:
        structure = StateMachineStructure()
        structure.machineName = self.state_machine.name
        for origin, destinations in self.state_machine.state_transitions.items():
            transition = StateMachineTransition()
            transition.origin = origin.__name__
            transition.destinations = [dest.__name__ for dest in destinations]
            structure.transitions.append(transition)
        self.structure_publisher.publish(structure)

    def publish_state(self) -> None:
        with self.state_machine.state_lock:
            cur_state = self.state_machine.current_state
        state = StateMachineStateUpdate()
        state.stateMachineName = self.state_machine.name
        state.state = str(cur_state)
        self.state_publisher.publish(state)

    def run_at_interval(self, func: Callable[[], None], update_hz: float):
        desired_loop_time = 1.0 / update_hz
        while True:
            start_time = time.time()
            if self.__stop_event.is_set():
                break
            func()
            elapsed_time = time.time() - start_time
            if desired_loop_time - elapsed_time > 0:
                time.sleep(desired_loop_time - elapsed_time)
