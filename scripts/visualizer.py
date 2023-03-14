#!/usr/bin/env python3

from __future__ import annotations
import signal
import graphviz  # type: ignore
import time
from PyQt5.QtWidgets import *  # type: ignore
from PyQt5.QtCore import *  # type: ignore
from PyQt5.QtGui import QPainter  # type: ignore
from PyQt5.QtSvg import QSvgRenderer  # type:ignore

import rospy
import sys
from smach_msgs.msg import SmachContainerStatus, SmachContainerStructure
from threading import Lock
from dataclasses import dataclass
from typing import Optional, List, Dict


STRUCTURE_TOPIC = "/smach/container_structure"
STATUS_TOPIC = "/smach/container_status"


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
        self.previous_state: str = ""
        self.needs_redraw: bool = True

    def set_active_state(self, active_state):
        """
        sets the state specified to be active (thread safe)
        """
        with self.mutex:
            if active_state != self.cur_active and active_state in self.states:
                self.previous_state = self.cur_active
                self.cur_active = active_state
                self.needs_redraw = True
                now = rospy.Time.now()
                rospy.loginfo(
                    f"Current time: {now} Previous state: {self.previous_state} Current State: { self.cur_active}"
                )

    def _rebuild(self, structure: SmachContainerStructure):
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
        checks if the structure passed as input matches the structure already represented (thread safe)
        """
        with self.mutex:
            if structure == self.structure:
                return False
            else:
                self._rebuild(structure)
                self.structure = structure

    def container_status_callback(self, status: SmachContainerStatus):
        self.set_active_state(status.active_states[0])

    def container_structure_callback(self, structure: SmachContainerStructure):
        self.check_rebuild(structure)


class GUI(QWidget):  # type: ignore
    def __init__(self, state_machine_instance, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.label: QLabel = QLabel()  # type: ignore
        self.timer: QTimer = QTimer()  # type: ignore
        self.renderer: QSvgRenderer = QSvgRenderer()
        self.timer.timeout.connect(self.update)
        self.timer.start(1)
        self.graph: Optional[graphviz.Digraph] = None
        self.img = None
        self.state_machine: StateMachine = state_machine

    def paintEvent(self, event):
        painter = QPainter(self)
        if self.img is not None:
            self.renderer.load(self.img)
        self.resize(self.renderer.defaultSize())
        self.renderer.render(painter)

    def update(self):
        with self.state_machine.mutex:
            if self.graph is None or self.state_machine.needs_redraw:
                self.graph = graphviz.Digraph(comment="State Machine Diagram", format="svg")
                for state_name in self.state_machine.states.keys():
                    color = "red" if self.state_machine.cur_active == state_name else "black"
                    self.graph.node(state_name, color=color)

                for state in self.state_machine.states.values():
                    for child in state.children:
                        self.graph.edge(state.name, child.name)

                self.state_machine.needs_redraw = False

        self.img = self.graph.pipe()
        self.repaint()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    state_machine = StateMachine()
    rospy.init_node("smach_visualizer", anonymous=False, disable_signals=True, log_level=rospy.INFO)
    rospy.Subscriber(
        STRUCTURE_TOPIC,
        SmachContainerStructure,
        state_machine.container_structure_callback,
    )
    rospy.Subscriber(
        STATUS_TOPIC,
        SmachContainerStatus,
        state_machine.container_status_callback,
    )
    app = QApplication([])  # type: ignore
    g = GUI(state_machine)
    g.show()
    app.exec_()
