from state_machine import StateMachine
from state import State
import rospy
from mrover.msg import StateMachineStructure, StateMachineTransition, StateMachineStateUpdate
import threading
from typing import Callable
import time

class StatePublisher:

    structure_publisher: rospy.Publisher
    state_publisher: rospy.Publisher
    state_machine: StateMachine
    __struct_thread: threading.Thread
    __state_thread: threading.Thread
    __stop_lock: threading.Lock()
    __stop: bool

    def __init__(self, state_machine: StateMachine, structure_pub_topic: str,
                structure_update_rate_hz: float, state_pub_topic: str, state_update_rate_hz: float):
        self.state_machine = state_machine
        self.structure_publisher = rospy.Publisher(structure_pub_topic, StateMachineStructure, queue_size=1)
        self.state_publisher = rospy.Publisher(state_pub_topic, StateMachineStateUpdate, queue_size=1)
        self.__stop_lock = threading.Lock()
        self.__struct_thread = threading.Thread(target=self.run_at_interval, args=(self.publish_structure, structure_update_rate_hz))
        self.__state_thread = threading.Thread(target=self.run_at_interval, args=(self.publish_state, state_update_rate_hz))
        self.__stop = False
    
    def stop(self) -> None:
        with self.__stop_lock:
            self.__stop = True
        
    def publish_structure(self) -> None:
        structure = StateMachineStructure()
        structure.machineName = self.state_machine.name
        for origin, destinations in self.state_machine.state_transitions.items():
            transition = StateMachineTransition()
            transition.origin = str(origin)
            transition.destinations = [str(dest) for dest in destinations]
            structure.transitions.append(transition)
        structure_publisher.publish(structure)
    
    def publish_state(self) -> None:
        with self.state_machine.state_lock:
            cur_state = self.state_machine.current_state
        state = StateMachineStateUpdate()
        state.machineName = self.state_machine.name
        state.state = str(cur_state)
        state_publiser.publish(state)
    
    def run_at_interval(self, func: Callable[[StatePublisher, [None]]], update_hz: float):
        desired_loop_time = 1.0 / update_hz
        while True:
            start_time = time.time()
            with self.__stop_lock:
                if self.__stop: break
            self.func()
            time.sleep(desired_loop_time - (time.time() - start_time))


        