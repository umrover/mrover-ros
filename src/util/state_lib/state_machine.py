import time
from collections import defaultdict
from dataclasses import dataclass
from enum import Enum
from threading import Lock
from typing import DefaultDict, Set, List, Callable, Any, Optional

from util.state_lib.state import State, ExitState


class LogLevel(Enum):
    OFF = 0
    DEBUG = 1
    VERBOSE = 2


@dataclass
class TransitionRecord:
    time: float
    origin_state: str
    dest_state: str


class StateMachine:
    current_state: State
    state_lock: Lock
    state_transitions: DefaultDict[type[State], Set[type[State]]]
    transition_log: List[TransitionRecord]
    context: Any
    name: str
    off_lambda: Optional[Callable[[Any], bool]]
    off_state: Optional[State]
    log_level: LogLevel
    logger: Callable[[str], None]
    on: bool
    onLock: Lock

    def __init__(
        self,
        initial_state: State,
        name: str,
        log_level: LogLevel = LogLevel.DEBUG,
        logger: Callable[[str], None] = print,
    ):
        self.current_state = initial_state
        self.state_lock = Lock()
        self.state_transitions = defaultdict(set)
        self.state_transitions[type(self.current_state)] = set()
        self.transition_log: List[TransitionRecord] = []
        self.context = None
        self.name = name
        self.off_lamdba = None
        self.off_state = None
        self.log_level = log_level
        self.logger = logger
        self.on = True
        self.onLock = Lock()

    def __update(self):
        with self.state_lock:
            current_state = self.current_state
        if self.log_level == LogLevel.VERBOSE:
            self.logger(f"{self.name} state machine, current state = {str(current_state)}")
        if self.off_lambda is not None and self.off_lambda(self.context) and self.off_state is not None:
            next_state = self.off_state
        else:
            next_state = current_state.on_loop(self.context)
        if type(next_state) not in self.state_transitions[type(current_state)]:
            raise Exception(f"Invalid transition from {current_state} to {next_state}")
        if type(next_state) is not type(current_state):
            if self.log_level == LogLevel.DEBUG or self.log_level == LogLevel.VERBOSE:
                self.logger(f"{self.name} state machine, transistioning to {str(next_state)}")
            current_state.on_exit(self.context)
            self.transition_log.append(TransitionRecord(time.time(), str(current_state), str(next_state)))
            with self.state_lock:
                self.current_state = next_state
                self.current_state.on_enter(self.context)

    def stop(self):
        with self.onLock:
            self.on = False

    def run(self, update_rate: float = float("inf"), warning_handle: Callable = print):
        """
        Runs the state machine until it returns an ExitState.
        Aims for as close to update_rate_hz, updates per second
        :param update_rate: targeted updates per second
        """
        target_loop_time = None if update_rate == float("inf") else (1.0 / update_rate)
        self.current_state.on_enter(self.context)
        is_on = True

        while is_on:
            start = time.time()
            self.__update()
            if isinstance(self.current_state, ExitState):
                break
            with self.onLock:
                is_on = self.on
            elapsed_time = time.time() - start
            if target_loop_time is not None and elapsed_time < target_loop_time:
                time.sleep(target_loop_time - elapsed_time)
            elif target_loop_time is not None and elapsed_time > target_loop_time:
                warning_handle(
                    f"[WARNING] state machine loop overran target loop time by {elapsed_time - target_loop_time} s"
                )

    def add_transition(self, state_from: State, state_to: State) -> None:
        self.state_transitions[type(state_from)].add(type(state_to))

    def add_transitions(self, state_from: State, states_to: List[State]) -> None:
        for state_to in states_to:
            self.add_transition(state_from, state_to)
        self.add_transition(state_from, state_from)
        if self.off_state is not None:
            self.add_transition(state_from, self.off_state)

    def set_context(self, context: Any):
        self.context = context

    def configure_off_switch(self, off_state: State, off_lambda: Callable[[Any], bool]):
        if type(off_state) not in self.state_transitions:
            raise Exception("Attempted to configure an Off State that doesn't exist")
        self.off_state = off_state
        self.off_lambda = off_lambda
        for _, to_states in self.state_transitions.items():
            to_states.add(type(off_state))
