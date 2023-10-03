from .state import State, ExitState
from typing import Dict, Set, List
import time
from dataclasses import dataclass

@dataclass
class TransitionRecord:
    time: float
    origin_state: str
    dest_state: str

class StateMachine:
    def __init__(self, initial_state: State):
        self.current_state = initial_state
        self.state_transitions: Dict[type[State], Set[type[State]]]  = {}
        self.transition_log: List[TransitionRecord] = []
        self.context = None
    
    def __update(self):
        next_state = self.current_state.on_loop(self.context)
        if type(next_state) not in self.state_transitions[type(self.current_state)]:
            raise Exception(f"Invalid transition from {self.current_state} to {next_state}")
        if type(next_state) is not type(self.current_state):
            self.current_state.on_exit(self.context)
            self.transition_log.append(TransitionRecord(time.time(), str(self.current_state), str(next_state)))
            self.current_state = next_state
            self.current_state.on_enter(self.context)
    
    def run(self):
        self.current_state.on_enter(self.context)
        while True:
            self.__update()
            if type(self.current_state) is ExitState:
                break
    
    def add_transition(self, state_from: State, state_to: State):
        if type(state_from) not in self.state_transitions:
            self.state_transitions[type(state_from)] = set()
        self.state_transitions[type(state_from)].add(type(state_to))
    
    def set_context(self, context):
        self.context = context

