from .state import State, ExitState
from typing import Dict, Set, List, Callable
import time
from dataclasses import dataclass
from threading import Lock

@dataclass
class TransitionRecord:
    time: float
    origin_state: str
    dest_state: str

class StateMachine:
    def __init__(self, initial_state: State, name: str):
        self.current_state = initial_state
        self.state_lock = Lock()
        self.state_transitions: Dict[type[State], Set[type[State]]]  = {}
        self.transition_log: List[TransitionRecord] = []
        self.context = None
        self.name = name
    
    def __update(self):
        with self.state_lock:
            current_state = self.current_state
        next_state = current_state.on_loop(self.context)
        if type(next_state) not in self.state_transitions[type(current_state)]:
            raise Exception(f"Invalid transition from {current_state} to {next_state}")
        if type(next_state) is not type(current_state):
            current_state.on_exit(self.context)
            self.transition_log.append(TransitionRecord(time.time(), str(current_state), str(next_state)))
            with self.state_lock:
                self.current_state = next_state
                self.current_state.on_enter(self.context)
    
    def run(self, update_rate: float = float('inf'), warning_handle: Callable = print):
        '''
        Runs the state machine until it returns an ExitState. 
        Aims for as close to update_rate_hz, updates per second
        :param update_rate_z (float): targetted updates per second
        '''
        target_loop_time = None if update_rate == float('inf') else (1.0 / update_rate)
        self.current_state.on_enter(self.context)
        while True:
            start = time.time()
            self.__update()
            if type(self.current_state) == ExitState:
                break
            elapsed_time = time.time() - start
            if target_loop_time is not None and elapsed_time < target_loop_time:
                time.sleep(target_loop_time - elapsed_time)
            elif target_loop_time is not None and elapsed_time > target_loop_time:
                warning_handle(f"[WARNING] state machine loop overran target loop time by {elapsed_time - target_loop_time} s")
                
    
    def add_transition(self, state_from: State, state_to: State):
        if type(state_from) not in self.state_transitions:
            self.state_transitions[type(state_from)] = set()
        self.state_transitions[type(state_from)].add(type(state_to))
    
    def set_context(self, context):
        self.context = context

