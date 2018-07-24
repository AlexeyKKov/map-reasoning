import logging
import time

from grounding import pddl_grounding
from grounding import json_grounding
from search.mapsearch import Map_search
from agent.messagen import Tmessage
from grounding.sign_task import Task


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
        search = Map_search(task)
        self.solution = search.search_plan()

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



