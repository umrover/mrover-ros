from __future__ import annotations
import sys
import unittest
from util.state_lib.state_machine import StateMachine
from util.state_lib.state import State, ExitState


var = 0

class ForwardState(State):
    def on_enter(self, context):
        print("Entering ForwardState")
    
    def on_exit(self, context):
        print("Exiting ForwardState")
    
    def on_loop(self, context) -> State:
        global var
        print(f"Looping ForwardState var: {var}")
        var += 1
        if var == 10:
            return BackwardState()
        return ForwardState()

class BackwardState(State):
    def on_enter(self, context):
        print("Entering BackwardState")

    def on_exit(self, context):
        print("Exiting BackwardState")

    def on_loop(self, context) -> State:
        global var
        print(f"Looping BackwardState var: {var}")
        var -= 1
        if var == 0:
            return ExitState()
        return BackwardState()

if __name__ == "__main__":
    sm = StateMachine(ForwardState())
    sm.add_transition(ForwardState(), BackwardState())
    sm.add_transition(BackwardState(), ForwardState())
    sm.add_transition(BackwardState(), ExitState())
    sm.add_transition(ForwardState(), ForwardState())
    sm.add_transition(BackwardState(), BackwardState())
    sm.run()