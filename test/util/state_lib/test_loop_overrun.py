from __future__ import annotations
import sys
import unittest
from util.state_lib.state_machine import StateMachine
from util.state_lib.state import State, ExitState
import unittest
import time


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
        time.sleep(1)
        if context.var == 0:
            return ExitState()
        return BackwardState()


from dataclasses import dataclass


@dataclass
class Context:
    var: int = 0
    forward_loop_count: int = 0
    backward_loop_count: int = 0


@dataclass
class WarningHandle:
    got_warning: bool = False

    def set_warning(self, s):
        self.got_warning = True


class TestLoopOverrun(unittest.TestCase):
    def test_loop_overrun(self):
        context = Context()
        sm = StateMachine[Context](ForwardState(), "LoopOverrun", context)
        sm.add_transition(ForwardState(), BackwardState())
        sm.add_transition(BackwardState(), ForwardState())
        sm.add_transition(BackwardState(), ExitState())
        sm.add_transition(ForwardState(), ForwardState())
        sm.add_transition(BackwardState(), BackwardState())
        wh = WarningHandle()
        sm.run(10, wh.set_warning)
        self.assertTrue(wh.got_warning)
        tlog = sm.transition_log
        timeless = [(t.origin_state, t.dest_state) for t in tlog]
        expected = [("ForwardState", "BackwardState"), ("BackwardState", "ExitState")]
        for t, e in zip(timeless, expected):
            self.assertEqual(t, e)
        self.assertEqual(context.forward_loop_count, 3)
        self.assertEqual(context.backward_loop_count, 3)


if __name__ == "__main__":
    import rostest

    rostest.rosrun("mrover", "SimpleStateLibraryTaste", TestLoopOverrun)
