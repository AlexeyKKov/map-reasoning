from agent.agent_grounding import Agent
from multiprocessing import Pool
from agent.messagen import reconstructor
import logging


class Manager:

    def __init__(self, agents, problem, logic, saveload, gazebo):
        self.agents = agents
        self.problem = problem
        self.saveload = saveload
        self.logic = logic
        self.solution = []
        self.gazebo = gazebo
        self.finished = None


    # send task for agent to start searching
    def agent_start(self, agent, port, others):
        saved = agent.search_solution(port, others)
        if saved:
            self.finished = True
        return True

    # create a pool of workers (1 per agent) and wait for their plans.
    def manage_agents(self):
        # binding a server socket for solution
        clagents = []
        port = 9097
        import socket
        serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        serversocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        serversocket.bind(('', port))
        serversocket.listen(5)

        for agent in self.agents:
            agent = Agent(agent, self.agents, self.problem, self.logic, self.saveload, self.gazebo) # create Agents with info about a problem
            port += 1
            clagents.append([agent, port])
        for current_agent in clagents:
            others = [[agent[0].name, agent[1]] for agent in clagents if not agent is current_agent]
            current_agent.insert(2, others)

        pool = Pool(processes=len(clagents)) # make a pool with agents
        multiple_results = [pool.apply_async(self.agent_start, (agent, port, others)) for agent, port, others in
                            clagents]

        while True:
            # waiting plans from agents
            (clientsocket, address) = serversocket.accept()
            solution = clientsocket.recv(1024)
            solution = solution.decode()

            if solution:
                self.solution.append(solution)
            else:
                logging.info("Solution is not found by agents")
            print('connected:', address)
            if len(self.solution) == len(clagents):
                agent, self.solution = auction(self.solution)
                # send best solution forward to agents
                clientsocket.send(self.solution.encode('utf-8'))
                break

        clientsocket.close()

        if multiple_results:
            pool.close()

        # STOP THE PARENT PROCESS and Let Agents finish their goals
        import time
        max_time = 10
        start_time = 0
        while not self.finished:
            time.sleep(1)
            start_time += 1
            if start_time >= max_time:
                break

        return self.solution

def auction(solutions):
    plans = {}
    auct = {}
    maxim = 1
    for sol in solutions:
        agent, plan = reconstructor(sol)
        plans[agent] = plan
    for agent, plan in plans.items():
        if not plan in auct:
            auct[plan] = 1
        else:
            iter = auct[plan]
            auct[plan] = iter+1
            if iter+1 > maxim:
                maxim = iter+1

    plan = [plan for plan, count in auct.items() if count==maxim][0]

    agents = []
    for agent, pl in plans.items():
        if pl == plan:
            agents.append(agent)
    return agents[0], plan
















