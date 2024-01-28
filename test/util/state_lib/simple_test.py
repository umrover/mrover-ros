from __future__ import annotations
import sys
import unittest
from util.state_lib.state_machine import StateMachine
from util.state_lib.state import State, ExitState
import unittest


class ForwardState(State):
    def on_enter(self, context):
        pass

    def on_exit(self, context):
        pass

    def on_loop(self, context) -> State:
        context.var += 1
        context.forward_loop_count += 1
        if context.var == 3:
            return BackwardState()
        return ForwardState()


class BackwardState(State):
    def on_enter(self, context):
        pass

    def on_exit(self, context):
        pass

    def on_loop(self, context) -> State:
        context.var -= 1
        context.backward_loop_count += 1
        if context.var == 0:
            return ExitState()
        return BackwardState()


from dataclasses import dataclass


@dataclass
class Context:
    var: int = 0
    forward_loop_count: int = 0
    backward_loop_count: int = 0


class TestSimpleStateMachine(unittest.TestCase):
    def test_simple(self):
        context = Context()
        sm = StateMachine[Context](ForwardState(), "SimpleStateMachine", context)
        sm.add_transition(ForwardState(), BackwardState())
        sm.add_transition(BackwardState(), ForwardState())
        sm.add_transition(BackwardState(), ExitState())
        sm.add_transition(ForwardState(), ForwardState())
        sm.add_transition(BackwardState(), BackwardState())
        sm.run()
        tlog = sm.transition_log
        timeless = [(t.origin_state, t.dest_state) for t in tlog]
        expected = [("ForwardState", "BackwardState"), ("BackwardState", "ExitState")]
        for t, e in zip(timeless, expected):
            self.assertEqual(t, e)
        self.assertEqual(context.forward_loop_count, 3)
        self.assertEqual(context.backward_loop_count, 3)


if __name__ == "__main__":
    import rostest

    rostest.rosrun("mrover", "SimpleStateLibraryTaste", TestSimpleStateMachine)
