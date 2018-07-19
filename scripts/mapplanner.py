#!/usr/bin/env python3
import argparse
import logging
import os
import json
import re
import time

from grounding.json_grounding import Problem
from pddl.parser import Parser
from connection.manager import Manager

SOLUTION_FILE_SUFFIX = '.soln'


def clarify_problem(path, number, logic):
    domain = ''
    task = ''
    if logic == 'spatial':
        if logic not in path:
            raise Exception('Spatial logic only for spatial benchmarks!')
        domain = 'domain.json'
        task = 'task' + number + '.json'
    elif logic == 'classic':
        domain = 'domain.pddl'
        task = 'task' + number + '.pddl'
    print(os.getcwd()+'/'+path)
    if not domain in os.listdir(path):
        raise Exception('domain not found!')
    else:
        domain = path + domain
    if not task in os.listdir(path):
        raise Exception('task not found!')
    else:
        problem = path + task

    return domain, problem

def _parse(domain_file, problem_file):
    # Parsing
    parser = Parser(domain_file, problem_file)
    logging.info('Parsing Domain {0}'.format(domain_file))
    domain = parser.parse_domain()
    logging.info('Parsing Problem {0}'.format(problem_file))
    problem = parser.parse_problem(domain)
    logging.debug(domain)
    logging.info('{0} Predicates parsed'.format(len(domain.predicates)))
    logging.info('{0} Actions parsed'.format(len(domain.actions)))
    logging.info('{0} Objects parsed'.format(len(problem.objects)))
    logging.info('{0} Constants parsed'.format(len(domain.constants)))
    return problem

def agent_manager(agents, problem, logic, saveload, gazebo = False):
    manager = Manager(agents, problem, logic, saveload, gazebo)
    solution = manager.manage_agents()
    return solution

def search_classic(domain_file, problem_file, logic, saveload):
    def action_agents(problem):
        agents = set()
        for _, action in problem.domain.actions.items():
            for ag in action.agents:
                for obj, type in problem.objects.items():
                    if type.name == ag:
                        agents.add(obj)
        return agents

    problem = _parse(domain_file, problem_file)
    act_agents = action_agents(problem)
    logging.info('Agents found in actions: {0}'.format(len(act_agents)))
    agents = set()
    if problem.constraints:
        if len(act_agents):
            agents |= act_agents
        else:
            for constr in problem.constraints:
                agents.add(constr)
            logging.info('Agents found in constraints: {0}'.format(len(agents)))
    elif act_agents:
        agents |= act_agents
    else:
        agents.add('I')
        logging.info('Only 1 agent plan')


    return agent_manager(agents, problem, logic, saveload)

def search_spatial(domain_file, problem_file, logic = 'spatial', saveload = False, gazebo = False):

    with open(problem_file) as data_file1:
        problem_parsed = json.load(data_file1)
    with open(domain_file) as data_file2:
        signs_structure = json.load(data_file2)

    problem = Problem(signs_structure, problem_parsed, None)

    return agent_manager(problem_parsed['agents'], problem, logic, saveload, gazebo)

def find_solution(domain, problem, LogicType, saveload, gazebo):
    if LogicType == 'spatial':
        solution = search_spatial(domain, problem, LogicType, saveload, gazebo)
    elif LogicType == 'classic':
        if gazebo:
            raise Exception('Only spatial logic support gazebo visualization!')
        solution = search_classic(domain, problem, LogicType, saveload)
    else:
        raise Exception('Logic type %s is not supported!' % LogicType)
    return solution

if __name__ == '__main__':

    # Commandline parsing
    log_levels = ['debug', 'info', 'warning', 'error']
    logic_type = ['spatial', 'classic']

    argparser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    argparser.add_argument(dest='benchmark')
    argparser.add_argument(dest='task_number')
    argparser.add_argument('-g', '--gazebo', action='store_true')
    argparser.add_argument('-l', '--loglevel', choices=log_levels,
                           default='info')
    argparser.add_argument('-lt', '--LogicType', choices=logic_type,
                           default='spatial')
    argparser.add_argument('-s', '--save', action='store_true')
    argparser.add_argument('-w', '--load', action='store_true')

    args = argparser.parse_args()

    rootLogger = logging.getLogger()
    logFormatter = logging.Formatter("%(asctime)s %(levelname)-8s  %(message)s")
    consoleHandler = logging.StreamHandler()
    consoleHandler.setFormatter(logFormatter)
    rootLogger.addHandler(consoleHandler)
    fileHandler = logging.FileHandler('pmaplanner.log')
    fileHandler.setFormatter(logFormatter)
    rootLogger.addHandler(fileHandler)
    rootLogger.setLevel(args.loglevel.upper())

    domain, problem = clarify_problem(args.benchmark, args.task_number, args.LogicType)

    solution = find_solution(domain, problem, args.LogicType, args.save, args.gazebo)

    # solution = search_plan(args.domain, args.problem, args.save, args.load)
    #
    # if solution is None:
    #     logging.warning('No solution could be found')
    # else:
    #     solution_file = args.problem + SOLUTION_FILE_SUFFIX
    #     logging.info('Plan length: %s' % len(solution))
    #     with open(solution_file, 'w') as file:
    #         for name, op in solution:
    #             print(name, op, file=file)
