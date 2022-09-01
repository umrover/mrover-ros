from __future__ import annotations

import graphviz
import time
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import QPainter
from PyQt5.QtSvg import QSvgRenderer

import rospy
from smach_msgs.msg import SmachContainerStatus, SmachContainerStructure
from threading import Lock
from dataclasses import dataclass
from typing import Optional, List, Dict


@dataclass
class State:
    name: str
    children: List[State]


class StateMachine:
    def __init__(self):
        self.states: Dict[str, State] = {}
        self.structure: Optional[SmachContainerStructure] = None
        self.mutex: Lock = Lock()
        self.cur_active: str = ""
        self.needs_redraw: bool = True

    def set_active_state(self, active_state):
        """
        sets the state specified to be active
        """
        if active_state != self.cur_active and active_state in self.states:
            self.cur_active = active_state
            self.needs_redraw = True

    def rebuild(self, structure: SmachContainerStructure):
        """
        rebuilds the state dictionary with a new structure message
        """
        self.states = {child: State(child, []) for child in structure.children}
        for start, end in zip(structure.outcomes_from, structure.outcomes_to):
            if end != "None":
                self.states[start].children.append(self.states[end])
        self.needs_redraw = True

    def check_rebuild(self, structure: SmachContainerStructure):
        """
        checks if the structure passed as input matches the structure already represented
        """
        if structure == self.structure:
            return False
        else:
            self.rebuild(structure)


state_machine = StateMachine()


def container_status_callback(status: SmachContainerStatus):
    with state_machine.mutex:
        state_machine.set_active_state(status.active_states[0])


def container_structure_callback(structure: SmachContainerStructure):
    with state_machine.mutex:
        state_machine.check_rebuild(structure)


class GUI(QWidget):
    def __init__(self):
        super().__init__()
        self.label = QLabel()
        self.timer = QTimer()
        self.renderer = QSvgRenderer()
        self.prev_time = time.time()
        self.timer.timeout.connect(self.update)
        self.timer.start(1)
        self.graph = None
        self.img = None

    def paintEvent(self, event):
        painter = QPainter(self)
        if self.img is not None:
            self.renderer.load(self.img)
        self.resize(self.renderer.defaultSize())
        self.renderer.render(painter)

    def update(self):

        with state_machine.mutex:

            print(time.time() - self.prev_time)
            self.prev_time = time.time()

            if self.graph is None or state_machine.needs_redraw:
                self.graph = graphviz.Digraph(comment="State Machine Diagram", format="svg")
                for state_name in state_machine.states.keys():
                    color = "red" if state_machine.cur_active == state_name else "black"
                    self.graph.node(state_name, color=color)

                # not sure if I can just do this in the above loop
                for state in state_machine.states.values():
                    for child in state.children:
                        self.graph.edge(state.name, child.name)

                state_machine.needs_redraw = False

        self.img = self.graph.pipe()
        self.repaint()


if __name__ == "__main__":
    rospy.init_node("smach visualizer", anonymous=False, disable_signals=True, log_level=rospy.INFO)
    rospy.Subscriber("/server_name/smach/container_structure", SmachContainerStructure, container_structure_callback)
    rospy.Subscriber("/server_name/smach/container_status", SmachContainerStatus, container_status_callback)
    app = QApplication([])
    g = GUI()
    g.show()
    app.exec()
