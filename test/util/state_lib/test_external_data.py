from util.state_lib.state_machine import StateMachine
from util.state_lib.state import State, ExitState
import random
from threading import Thread, Lock
import time

"""
Test multi-threaded program that has an external thread that feeds a resource that the
Context object queries. Not a unit-test but run manually to ensure relatively predicatble
behavior
"""


class Context:
    def __init__(self):
        self.stateCapture = ExternalStateCapture()

    def getTrigger(self):
        self.stateCapture.triggerLock.acquire()
        trigger = self.stateCapture.trigger
        self.stateCapture.triggerLock.release()
        return trigger


class ExternalStateCapture:
    def random_loop(self):
        # run forever and with probability 0.5 flip the trigger
        while True:
            if random.random() < 0.5:
                self.triggerLock.acquire()
                self.trigger = not self.trigger
                self.triggerLock.release()
                time.sleep(1)

    def __init__(self):
        self.triggerLock = Lock()
        self.trigger = False


class WaitingState(State):
    def __init__(self):
        super().__init__()

    def on_enter(self, context):
        print("Waiting for trigger")

    def on_loop(self, context):
        if context.getTrigger():
            return RunningState()
        return self

    def on_exit(self, context):
        print("Triggered!")


class RunningState(State):
    def __init__(self):
        super().__init__()

    def on_enter(self, context):
        print("Running")

    def on_loop(self, context):
        if not context.getTrigger():
            return WaitingState()
        return self

    def on_exit(self, context):
        print("Stopped")


if __name__ == "__main__":
    context = Context()
    sm = StateMachine[Context](WaitingState(), "RandomForeverStateMachine", context)
    sm.add_transition(WaitingState(), RunningState())
    sm.add_transition(RunningState(), WaitingState())
    sm.add_transition(RunningState(), RunningState())
    sm.add_transition(WaitingState(), WaitingState())
    thread = Thread(target=context.stateCapture.random_loop)
    thread.start()
    sm.run()
