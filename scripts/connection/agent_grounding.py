import logging
import time
import re

from grounding import pddl_grounding
from grounding import json_grounding
from search.mapsearch import map_search
from connection.messagen import Tmessage
from grounding.sign_task import Task
from gazebo_adapter.gazebo_tools import TurtleBot, Processer

#ROS libs
import rospy
from std_srvs.srv import Empty

class Agent:
    def __init__(self, name, subjects, problem,logic, saveload, gazebo):
        self.name = name
        self.subjects = subjects
        self.problem = problem
        self.is_load = saveload
        self.solution = []
        self.types = ['help_request', 'Approve', 'Broadcast']
        self.logic = logic
        self.gazebo = gazebo
        self.final_solution = ''

    # Grounding tasks
    def load_sw(self):
        logging.info('Grounding start: {0}'.format(self.problem.name))
        if self.logic == 'classic':
            if self.is_load:
                signs = Task.load_signs(self.name)
                task = pddl_grounding.ground(self.problem, self.name, self.subjects, self.logic, signs)
            elif not self.is_load:
                task = pddl_grounding.ground(self.problem, self.name, self.subjects, self.logic)
        elif self.logic == 'spatial':
            task = json_grounding.spatial_ground(self.problem, self.name, self.logic)
        logging.info('Grounding end: {0}'.format(self.problem.name))
        logging.info('{0} Signs created'.format(len(task.signs)))
        return task
    # Gazebo publisher
    def gazebo_visualization(self):
        plan_to_gazebo = []
        pr = Processer(10, 10, 20)
        for action in self.final_solution.strip().split(" && "):
            if '(' in action and ')' in action:
                action = re.split('[\(\)]', action)
                coords = action[1].split(', ')
                new_coords = pr.to_gazebo(float(coords[0]), float(coords[1]))
                plan_to_gazebo.append((action[0],new_coords[0],new_coords[1],action[2]))
        # init ROS node
        rospy.init_node('signs_server')
        empty_call = rospy.ServiceProxy("/gazebo/unpause_physics", Empty)
        empty_call()
        tb = TurtleBot()
        prev_directions = []

        for action in plan_to_gazebo:
            logging.info('Trying to implement action {0}'.format(action[0]))
            try:
                if not prev_directions and 'move' not in action[0]:
                    prev_direct = self.problem.initial_state["agent-orientation"]
                    prev_directions.append(prev_direct)
                elif 'move' in action[0]:
                    prev_directions.append(action[3][1:])
                    prev_direct = prev_directions[-1]
                else:
                    prev_direct = prev_directions[-1]

                if  'move' in action[0]:
                    resp = tb.move(action[1], action[2], prev_direct)
                    if not resp:
                        raise Exception('Agent {0} can not implement a move action'.format(self.name))
                elif 'rotate' in action[0]:
                    prev_directions.append(action[3][1:])
                    resp = tb.rotate(action[3][1:], prev_direct)
                    if not resp:
                        raise Exception('Agent {0} can not implement a rotate action'.format(self.name))
                elif 'pick-up' in action[0]:
                    resp = tb.pickup(action, prev_direct)
                    if not resp:
                        raise Exception('Agent {0} can not implement a pick-up action'.format(self.name))
                elif 'put-down' in action[0]:
                    prev_direct = prev_directions[-1]
                    resp = tb.putdown(action, prev_direct)
                    if not resp:
                        raise Exception('Agent {0} can not implement a put-down action'.format(self.name))
            except rospy.ROSInterruptException:
                raise Exception('Somebody Interrupt Agent! Call 911!')

    def search_solution(self, port, others):
        task = self.load_sw()
        logging.info('Search start: {0}, Start time: {1}'.format(task.name, time.clock()))
        connection_sign = task.signs["Send"]
        cms = connection_sign.spread_up_activity_motor('significance', 1)
        method = None
        cm = None
        for sign, action in cms:
            for connector in sign.out_significances:
                if connector.in_sign.name == "They" and len(others) > 1:
                    method = action
                    pm = connector.out_sign.significances[1]
                    cm = pm.copy('significance', 'meaning')
                elif connector.in_sign.name != "They" and len(others) == 1:
                    method = action
                    cm = connector.out_sign.significances[1].copy('significance', 'meaning')
                elif len(others) == 0:
                    method = 'save_achievement'
        self.solution = map_search(task)

        self.solution.append((connection_sign.add_meaning(), method, cm, task.signs["I"]))

        mes = Tmessage(self.solution, self.name)
        message = getattr(mes, method)()

        import socket
        #send sol to server
        conn = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        conn.connect(('localhost', 9097))

        conn.send(message.encode("utf-8"))

        while True:
            auct_sol = conn.recv(1024)
            self.final_solution = auct_sol.decode()
            if self.solution:
                logging.info('Agent '+self.name+' got the final solution!')
            break

        conn.close()

        if self.is_load:
            task.save_signs(self.solution)


        if self.gazebo and self.final_solution:
            self.gazebo_visualization()
            #stop process finishing:
            while True:
                time.sleep(1)


