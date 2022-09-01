import graphviz
import time
from PyQt5.QtWidgets import *
from PyQt5.QtCore import *
from PyQt5.QtGui import QPainter
from PyQt5.QtSvg import QSvgRenderer

import rospy
from smach_msgs.msg import SmachContainerStatus,SmachContainerStructure
from threading import Lock
from dataclasses import dataclass, field
from typing import Optional

@dataclass
class State:
    name : str
    children : list #of states

@dataclass
class StateMachine:
    states : dict  = field(default_factory=dict)#maps string name to State
    structure : Optional[SmachContainerStructure] = None
    mutex : Lock = Lock()
    cur_active : str = ""
    needs_redraw : bool = True
    def set_active_state(self, active_state):
        """
        sets the state specified to be active
        """
        if active_state != self.cur_active and active_state in self.states:
            self.cur_active = active_state
            self.needs_redraw = True

    def rebuild(self, structure : SmachContainerStructure):
        """
        rebuilds the state dictionary with a new structure message
        """
        self.states = {child : State(child, []) for child in structure.children}
        for start, end in zip(structure.outcomes_from, structure.outcomes_to):
            if end != "None":
                self.states[start].children.append(self.states[end])
        self.needs_redraw = True


    def check_rebuild(self, structure : SmachContainerStructure):
        """
        checks if the structure passed as input matches the structure already represented
        """
        if structure == self.structure:
            return False
        else:
            self.rebuild(structure)

stateMachine = StateMachine()

def container_status_callback(status : SmachContainerStatus):
    #rospy.loginfo(status.active_states)
    stateMachine.mutex.acquire()
    stateMachine.set_active_state(status.active_states[0])
    stateMachine.mutex.release()

def container_structure_callback(structure : SmachContainerStructure):
    stateMachine.mutex.acquire()
    #rospy.loginfo(f"structure received\n path: {structure.path} \n  children: {structure.children}\n  internal outcomes: {structure.internal_outcomes}\n, outcomes from: {structure.outcomes_from} \n, outcomes to: {structure.outcomes_to}\n.")
    stateMachine.check_rebuild(structure)
    stateMachine.mutex.release()

class GUI(QWidget):
    def __init__(self):
        super().__init__()
        self.label = QLabel()
        self.timer = QTimer()
        self.renderer = QSvgRenderer()
        self.iter = 0
        self.node_color = "black"
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

        stateMachine.mutex.acquire()

        #print(time.time() - self.prev_time)
        self.prev_time = time.time()

        if self.graph is None or stateMachine.needs_redraw:
            self.graph = graphviz.Digraph(comment="State Machine Diagram", format = "svg")
            for state_name in stateMachine.states.keys():
                color = "red" if stateMachine.cur_active == state_name else "black"
                print(state_name)
                self.graph.node(state_name, color=color)
            
            #not sure if I can just do this in the above loop
            for state in stateMachine.states.values():
                for child in state.children:
                    print(state.name, child)

                    self.graph.edge(state.name, child.name)


            stateMachine.needs_redraw = False



        stateMachine.mutex.release()

        
        # self.iter += 1
        # if(self.iter % 10 == 0):
        #     if(self.node_color == "red"):
        #         self.node_color = "black"
        #     else:
        #         self.node_color = "red"

        # graph = graphviz.Digraph(comment="My Graph", format="svg")
        # graph.node("node A", color=self.node_color)
        # graph.node("node B")
        # graph.edge("node A", "node B", "edge label")
        # graph.edge("node B", "node A", "edge label 2")
        self.img = self.graph.pipe()
        self.repaint()


rospy.init_node('smach visualizer',anonymous=False, disable_signals=True,log_level=rospy.INFO)
rospy.Subscriber("/server_name/smach/container_structure", SmachContainerStructure, container_structure_callback)
rospy.Subscriber("/server_name/smach/container_status", SmachContainerStatus, container_status_callback)
#rospy.spin()
app = QApplication([])
g = GUI()
g.show()
app.exec()