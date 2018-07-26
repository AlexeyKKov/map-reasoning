import logging

import itertools
from copy import deepcopy, copy
import grounding.sign_task as st
from grounding.semnet import Sign
import random
from grounding.json_grounding import state_prediction
from grounding.json_grounding import define_situation
from grounding.json_grounding import signs_markup
from grounding.json_grounding import define_map
from grounding.json_grounding import update_situation




class Map_search():
    def __init__ (self, task):
        self.world_model = task.signs
        self.check_pm = task.goal_situation.meanings[1]
        self.active_pm = task.start_situation.meanings[1]
        self.constraints = task.constraints
        self.logic = task.logic
        self.active_map = task.map_precisions
        self.additions = task.additions
        self.exp_actions = []
        self.agents = set()
        self.I_sign = None
        self.I_obj = None
        if task.goal_map:
            self.check_map = task.goal_map.meanings[1]
        else:
            self.check_map = None
        self.MAX_ITERATION = 30
        logging.debug('Start: {0}'.format(self.check_pm.longstr()))
        logging.debug('Finish: {0}'.format(self.active_pm.longstr()))

    def search_plan(self):
        self.I_sign, self.I_obj, self.agents = self.__get_agents()
        plans = self._map_iteration(self.active_pm, self.active_map, iteration=0, current_plan=[])
        return plans

    def _logic_expand(self):
        pass

    def _map_iteration(self, active_pm, active_map, iteration, current_plan, prev_state = []):
        logging.debug('STEP {0}:'.format(iteration))
        logging.debug('\tSituation {0}'.format(active_pm.longstr()))
        if iteration >= self.MAX_ITERATION:
            logging.debug('\tMax iteration count')
            return None

        precedents = []
        plan_signs = []

        for name, sign in self.world_model.items():
            if name.startswith("action_"): plan_signs.append(sign)
            for index, cm in sign.meanings.items():
                if cm.includes('meaning', active_pm):
                    precedents.extend(cm.sign.spread_up_activity_act('meaning', 1))
                elif not cm.sign.significances and active_pm.includes('meaning', cm):
                    precedents.extend(cm.sign.spread_up_activity_act('meaning', 1))

        active_chains = active_pm.spread_down_activity('meaning', 4)
        active_signif = set()

        for chain in active_chains:
            pm = chain[-1]
            active_signif |= pm.sign.spread_up_activity_act('significance', 3)

        meanings = []
        for pm_signif in active_signif:
            chains = pm_signif.spread_down_activity('significance', 6)
            merged_chains = []
            for chain in chains:
                for achain in active_chains:
                    if chain[-1].sign == achain[-1].sign and len(chain) > 2 and chain not in merged_chains:
                        merged_chains.append(chain)
                        break
            scripts = self._generate_meanings(merged_chains)
            meanings.extend(scripts)
        applicable_meanings = []
        agent = None
        if not precedents:
            for agent, cm in meanings:
                result, checked = self._check_activity(cm, active_pm)
                if result:
                    applicable_meanings.append((agent, checked))
        else:
            for cm in precedents + meanings:
                if isinstance(cm, tuple):
                    agent = cm[0]
                    cm = cm[1]
                result, checked = self._check_activity(cm, active_pm)
                if result:
                    applicable_meanings.append((agent, checked))

        candidates = self._meta_check_activity(active_pm, applicable_meanings, [x for x, _, _, _, _ in current_plan], iteration, prev_state)

        if not candidates:
            logging.debug('\tNot found applicable scripts ({0})'.format([x for _, x, _, _, _ in current_plan]))
            return None

        logging.debug('\tFound {0} variants'.format(len(candidates)))
        final_plans = []

        logging.info("len of curent plan is: {0}. Len of candidates: {1}".format(len(current_plan), len(candidates)))

        for counter, name, script, ag_mask in candidates:
            logging.debug('\tChoose {0}: {1} -> {2}'.format(counter, name, script))
            plan = copy(current_plan)

            if self.logic == 'spatial':
                next_pm, next_map, prev_state, direction = self._step_generating(active_pm, active_map, script, agent, iteration, prev_state, True)
                ag_place = (prev_state[-1][2] - prev_state[-1][0]) // 2 + prev_state[-1][0], (
                            prev_state[-1][3] - prev_state[-1][1]) // 2 + prev_state[-1][1]
                plan.append((active_pm, name, script, ag_mask, (ag_place, direction)))
            else:
                plan.append((active_pm, name, script, ag_mask, None))
                next_pm = self._time_shift_forward(active_pm, script)
                next_map = None
                prev_state.append(active_pm)

            if next_pm.includes('meaning', self.check_pm):
                map_check = False
                if next_map:
                    if next_map.includes('meaning', self.check_map):
                        map_check = True
                else:
                    map_check = True
                if map_check is True:
                    final_plans.append(plan)
                    plan_actions = [x.sign.name for _, _, x, _, _ in plan]
                    logging.info("len of detected plan is: {0}".format(len(plan)))
                    logging.info(plan_actions)
            else:
                #Expanding logic
                self._logic_expand()
                recursive_plans = self._map_iteration(next_pm, next_map, iteration + 1, plan, prev_state)
                if recursive_plans:
                    final_plans.extend(recursive_plans)

        return final_plans

    def __get_agents(self):
        agent_back = set()
        I_sign = self.world_model['I']
        agent_back.add(I_sign)
        I_objects = [con.in_sign for con in I_sign.out_significances if con.out_sign.name == "I"]
        if I_objects:
            I_obj = I_objects[0]
        else:
            I_obj = None
        They_sign = self.world_model["They"]
        agents = They_sign.spread_up_activity_obj("significance", 1)
        for cm in agents:
            agent_back.add(cm.sign)
        return I_sign, I_obj, agent_back

    def _generate_meanings(self, chains):
        def __get_role_index(chain):
            index = 0
            rev_chain = reversed(chain)
            for el in rev_chain:
                if len(el.cause) == 0:
                    continue
                elif len(el.cause) == 1:
                    if len(el.cause[0].coincidences) ==1:
                        index = chain.index(el)
                    else:
                        return index
                else:
                    return index
            return None

        def __get_big_role_index(chain):
            index = None
            for el in chain:
                if len(el.cause) == 1:
                    if len(el.cause[0].coincidences) ==1:
                        index = chain.index(el)
                        break
                else:
                    continue
            if index:
                return index
            return None
        def __merge_predicates(predicates):
            merged = set()
            predics = copy(predicates)
            while predics:
                pred = predics.pop()
                for pr in predics:
                    if pr.name != pred.name:
                        if set(pred.signature) & set(pr.signature):
                            predics.remove(pr)
                            merged.add((pred, pr))
            return merged

        big_replace = {}

        replace_map = {}
        main_pm = None
        for chain in chains:
            role_index = __get_role_index(chain)
            if role_index:
                if not chain[role_index].sign in replace_map:
                    replace_map[chain[role_index].sign] = [chain[-1]]
                else:
                    if not chain[-1] in replace_map[chain[role_index].sign]:
                        replace_map[chain[role_index].sign].append(chain[-1])
            role_index = __get_big_role_index(chain)
            if role_index:
                if not chain[role_index].sign in big_replace:
                    big_replace[chain[role_index].sign] = [chain[role_index + 1]]
                else:
                    if not chain[role_index + 1] in big_replace[chain[role_index].sign]:
                        big_replace[chain[role_index].sign].append(chain[role_index + 1])
                main_pm = chain[0]

        connectors = [agent.out_meanings for agent in self.agents]

        main_pm_len = len(main_pm.cause) + len(main_pm.effect) + 2

        mapped_actions = {}
        for agent_con in connectors:
            for con in agent_con:
                if con.in_sign == main_pm.sign:
                    mapped_actions.setdefault(con.out_sign, set()).add(con.in_sign.meanings[con.in_index])

        new_map = {}
        rkeys = {el for el in replace_map.keys()}
        pms = []

        if self.constraints:
            replaced = dict()
            pms_names = []
            for ag, actions in mapped_actions.items():
                # # firstly full signed actions from experience
                for pm in actions.copy():
                    # # remove expanded actions
                    max_len = 0
                    for event in pm.cause:
                        if len(event.coincidences) > max_len:
                            max_len = len(event.coincidences)
                            if max_len > 1:
                                break
                    if max_len == 1:
                        actions.remove(pm)
                        continue
                    if len(pm.cause) + len(pm.effect) != main_pm_len:
                        continue
                    pm_signs = pm.get_signs()
                    role_signs = rkeys & pm_signs
                    if not role_signs:
                        actions.remove(pm)
                        if not pms:
                            pms.append((ag, pm))
                            pms_names.append({s.name for s in pm_signs})
                        else:
                            for _, pmd in copy(pms):
                                if pmd.resonate('meaning', pm):
                                    break
                            else:
                                pms.append((ag, pm))
                                pms_names.append({s.name for s in pm_signs})
                ## get characters from constr to replace
                agcall = ag.name
                if ag.name == 'I':
                    agcall = self.I_obj.name
                if not agcall in self.constraints:
                    continue
                else:
                    constrain = self.constraints[agcall]
                variants = []
                for constr_role, predicates in constrain.items():
                    merged = __merge_predicates(predicates)
                    for sets in merged:
                        consist = [set(), []]
                        for pred in sets:
                            for signa in pred.signature:
                                if signa[1][0].name in constr_role:
                                    if not signa[0] in consist[1]:
                                        consist[1].append(signa[0])
                                else:
                                    consist[0].add(signa[0])
                        for names in pms_names:
                            if consist[0] <= names and consist[1][0] in names:
                                break
                        else:
                            variants.append((constr_role, consist))
                for act in actions:

                    cm_sign_names = {si.name for si in act.get_signs()}
                    for var in variants:
                        if var[1][0] <= cm_sign_names:
                            cm = act.copy('meaning', 'meaning')
                            for rkey, rvalue in replace_map.items():
                                if rkey.name == var[0]:
                                    obj_pm = None
                                    for pm in rvalue:
                                        if pm.sign.name == var[1][1][0]:
                                            obj_pm = pm
                                            break
                                    obj_cm = obj_pm.copy('significance', 'meaning')
                                    cm.replace('meaning', rkey, obj_cm)
                                    replaced.setdefault(ag, []).append(cm)
                                    break
            if replaced:
                mapped_actions = replaced


        for agent, lpm in mapped_actions.items():
            for pm in lpm.copy():
                if len(pm.cause) + len(pm.effect) != main_pm_len:
                    continue
                pm_signs = set()
                pm_mean = pm.spread_down_activity('meaning', 3)
                for pm_list in pm_mean:
                    pm_signs |= set([c.sign for c in pm_list])
                role_signs = rkeys & pm_signs
                if not role_signs:
                    lpm.remove(pm)
                    if not pms:
                        pms.append((agent, pm))
                    else:
                        for _, pmd in copy(pms):
                            if pmd.resonate('meaning', pm):
                                break
                        else:
                            pms.append((agent, pm))
            old_pms = []

            for pm in lpm:
                if len(pm.cause) + len(pm.effect) != main_pm_len:
                    continue
                pm_signs = set()
                pm_mean = pm.spread_down_activity('meaning', 3)
                for pm_list in pm_mean:
                    pm_signs |= set([c.sign for c in pm_list])
                if pm_signs not in old_pms:
                    old_pms.append(pm_signs)
                else:
                    continue
                role_signs = rkeys & pm_signs
                for role_sign in role_signs:
                    new_map[role_sign] = replace_map[role_sign]

                for chain in pm_mean:
                    if chain[-1].sign in big_replace and not chain[-1].sign in new_map :
                        for cm in big_replace.get(chain[-1].sign):
                            if self.world_model['cell?x'] in cm.get_signs() and self.world_model['cell?y'] in cm.get_signs():
                                new_map[chain[-1].sign] = [cm]


                ma_combinations = self.mix_pairs(new_map)

                for ma_combination in ma_combinations:
                    cm = pm.copy('meaning', 'meaning')
                    breakable = False
                    for role_sign, obj_pm in ma_combination.items():
                        if obj_pm.sign in pm_signs:
                            breakable = True
                            break
                        obj_cm = obj_pm.copy('significance', 'meaning')
                        cm.replace('meaning', role_sign, obj_cm)
                    if breakable:
                        continue

                    for matr in cm.spread_down_activity('meaning', 6):
                        if matr[-1].sign.name == 'cell?y' or matr[-1].sign.name == 'cell?x':
                            celly = self.world_model['cell?y']
                            cellx = self.world_model['cell?x']
                            cell_y_change = ma_combination[celly].copy('meaning', 'meaning')
                            cm.replace('meaning', celly, cell_y_change)
                            cell_x_change = ma_combination[cellx].copy('meaning', 'meaning')
                            cm.replace('meaning', cellx, cell_x_change)
                            break

                    if not pms:
                        pms.append((agent, cm))
                    else:
                        for _, pmd in copy(pms):
                            if pmd.resonate('meaning', cm):
                                break
                        else:
                            pms.append((agent, cm))
                if len(old_pms) == 64:
                    break

        return pms

    def _check_activity(self, pm, active_pm):
        if len(pm.cause) and len(pm.effect):
            result = True
        else:
            result = False
        for event in pm.cause:
            for fevent in active_pm.cause:
                if event.resonate('meaning', fevent, True, self.logic):
                    break
            else:
                result = False
                break

        if not result:
            expanded = pm.expand('meaning')
            if not len(expanded.effect) == 0:
                return self._check_activity(expanded, active_pm)
            else:
                return False, pm
        return result, pm


    def long_relations(self, plans):
        logging.info("Choosing the plan for auction")
        min = len(plans[0])
        for plan in plans:
            if len(plan) < min:
                min = len(plan)
        plans = [plan for plan in plans if len(plan) == min]

        busiest = []
        for index, plan in enumerate(plans):
            previous_agent = ""
            agents = {}
            counter = 0
            plan_agents = []
            for action in plan:
                if action[3] not in agents:
                    agents[action[3]] = 1
                    previous_agent = action[3]
                    counter = 1
                    if not action[3] is None:
                        plan_agents.append(action[3].name)
                    else:
                        plan_agents.append(str(action[3]))
                elif not previous_agent == action[3]:
                    previous_agent = action[3]
                    counter = 1
                elif previous_agent == action[3]:
                    counter += 1
                    if agents[action[3]] < counter:
                        agents[action[3]] = counter
            # max queue of acts
            longest = 0
            agent = ""
            for element in range(len(agents)):
                item = agents.popitem()
                if item[1] > longest:
                    longest = item[1]
                    agent = item[0]
            busiest.append((index, agent, longest, plan_agents))
        cheap = []
        alternative = []
        cheapest = []
        longest = 0
        min_agents = 100

        for plan in busiest:
            if plan[2] > longest:
                longest = plan[2]

        for plan in busiest:
            if plan[2] == longest:
                if len(plan[3]) < min_agents:
                    min_agents = len(plan[3])

        for plan in busiest:
            if plan[3][0]:
                if plan[2] == longest and len(plan[3]) == min_agents and "I" in plan[3]:
                    plans_copy = copy(plans)
                    cheap.append(plans_copy.pop(plan[0]))
                elif plan[2] == longest and len(plan[3]) == min_agents and not "I" in plan[3]:
                    plans_copy = copy(plans)
                    alternative.append(plans_copy.pop(plan[0]))
            else:
                plans_copy = copy(plans)
                cheap.append(plans_copy.pop(plan[0]))
        if len(cheap) >= 1:
            cheapest.extend(random.choice(cheap))
        elif len(cheap) == 0 and len(alternative):
            logging.info("There are no plans in which I figure")
            cheapest.extend(random.choice(alternative))

        return cheapest

    def _meta_check_activity(self, active_pm, scripts, prev_pms, iteration, prev_state):
        heuristic = []
        if self.logic == 'spatial':
            for agent, script in scripts:
                estimation, cell_coords_new, new_x_y, \
                cell_location, near_loc, region_map = self._state_prediction(active_pm, script, agent, iteration)

                if not new_x_y['objects']['agent']['x'] in range(0, new_x_y['map_size'][0]) or \
                        not new_x_y['objects']['agent']['y'] in range(0, new_x_y['map_size'][1]):
                    break

                for prev in prev_pms:
                    if estimation.resonate('meaning', prev, False, False):
                        if cell_coords_new['cell-4'] in prev_state:
                            break
                else:
                    counter = 0
                    cont_region = None
                    future_region = None
                    for reg, cellz in cell_location.items():
                        if 'cell-4' in cellz:
                            cont_region = reg
                    agent_sign = self.world_model['agent']
                    for iner in self.check_map.get_iner(self.world_model['contain'], 'meaning'):
                        iner_signs = iner.get_signs()
                        if agent_sign in iner_signs:
                            for sign in iner_signs:
                                if sign != agent_sign and 'region' in sign.name:
                                    future_region = sign
                                    break
                    if future_region.name != cont_region:
                        ag_orient = estimation.get_iner(self.world_model['orientation'], 'meaning')[0]
                        iner_signs = ag_orient.get_signs()
                        dir_sign = None
                        for sign in iner_signs:
                            if sign != self.world_model["I"]:
                                dir_sign = sign
                        normal_dir = self.additions[2][cont_region][future_region.name][1]
                        stright = self.get_stright(estimation, dir_sign)
                        if dir_sign.name == normal_dir:
                            if stright:
                                counter = 0
                            else:
                                counter += 2
                        # for move action
                        elif cell_coords_new['cell-4'] != self.additions[0][iteration]['cell-4']:
                            counter += 1
                        else:
                            # check clously to goal region regions
                            closely_goal = [reg for reg, ratio in self.additions[2][future_region.name].items() if
                                            ratio[0] == 'closely']
                            closely_dirs = set()
                            if cont_region not in closely_goal:
                                for region in closely_goal:
                                    closely_dirs.add(self.additions[2][cont_region][region][1])
                                if dir_sign.name in closely_dirs:
                                    if stright:
                                        counter = 0
                                    else:
                                        counter += 1
                            else:
                                closely_dir = self.additions[2][cont_region][future_region.name][1]
                                if dir_sign.name == closely_dir:
                                    counter += 2

                    else:
                        est_events = [event for event in estimation.cause if "I" not in event.get_signs_names()]
                        ce_events = [event for event in self.check_pm.cause if "I" not in event.get_signs_names()]
                        for event in est_events:
                            for ce in ce_events:
                                if event.resonate('meaning', ce):
                                    counter += 1
                                    break

                    heuristic.append((counter, script.sign.name, script, agent))
        elif self.logic == 'classic':
            for agent, script in scripts:
                estimation = self._time_shift_forward(active_pm, script)
                for prev in prev_pms:
                    if estimation.resonate('meaning', prev, False, False):
                        break
                else:
                    counter = 0
                    for event in self._applicable_events(estimation):
                        for ce in self._applicable_events(self.check_pm):
                            if event.resonate('meaning', ce):
                                counter += 1
                                break
                    heuristic.append((counter, script.sign.name, script, agent))
        if heuristic:
            best_heuristics = max(heuristic, key=lambda x: x[0])
            return list(filter(lambda x: x[0] == best_heuristics[0], heuristic))
        else:
            return None

    def _applicable_events(self, pm, effect = False):
        applicable = []
        if effect:
            search_in_part = pm.effect
        else:
            search_in_part = pm.cause
        for event in search_in_part:
            if len(event.coincidences) == 1:
                flag = False
                for connector in event.coincidences:
                    if connector.out_sign in self.agents:
                        flag = True
                if flag:
                    continue
            applicable.append(event)
        return applicable


    def _state_prediction(self, active_pm, script, agent, iteration, flag=False):
        def __search_cm(events_list, signs):
            searched = {}
            for event in events_list:
                for conn in event.coincidences:
                    if conn.out_sign in signs:
                        searched.setdefault(conn.out_sign, []).append(conn.get_out_cm('meaning'))
            for s in signs:
                if not s in searched:
                    searched[s] = None
            return searched
        direction = None
        cell = None
        events = []
        fast_estimation = self._time_shift_forward_spat(active_pm, script)
        searched = __search_cm(fast_estimation, [self.world_model['orientation'], self.world_model['holding'], self.world_model['employment']] )

        employment = searched[self.world_model['employment']][0]
        holding = searched[self.world_model['holding']]
        if holding:
            holding = holding[0]
        orientation = searched[self.world_model['orientation']][0]
        for sign in orientation.get_signs():
            if sign != agent:
                direction = sign
                break
        for sign in employment.get_signs():
            if sign != agent:
                cell = sign
                break
        for ev in fast_estimation:
            if len(ev.coincidences) == 1:
                for con in ev.coincidences:
                    if con.out_sign.name == "I":
                        events.append(ev)
            elif not holding:
                if "I" in [s.name for s in ev.get_signs()]:
                    events.append(ev)
        agent_state = state_prediction(agent, direction, holding)
        cell_coords = deepcopy(self.additions[0][iteration][cell.name])
        new_x_y = deepcopy(self.additions[1][iteration])
        new_x_y['objects']['agent']['x'] = (cell_coords[2] - cell_coords[0]) // 2 + cell_coords[0]
        new_x_y['objects']['agent']['y'] = (cell_coords[3] - cell_coords[1]) // 2 + cell_coords[1]
        # for pick-up script
        if holding:
            block_name = [sign.name for sign in holding.get_signs() if 'block' in sign.name][0]
            if block_name in new_x_y['objects'].keys():
                new_x_y['objects'][block_name]['y'] = new_x_y['objects']['agent']['y']
                new_x_y['objects'][block_name]['x'] = new_x_y['objects']['agent']['x']
            else:
                block = {}
                block[block_name] = {'x': new_x_y['objects']['agent']['x'], 'y': new_x_y['objects']['agent']['y'], 'r': 5}
                new_x_y['objects'].update(block)
                # print()
        # for put-down script
        if script.sign.name == 'put-down':
            block_name = [sign.name for sign in script.get_iner(self.world_model['holding'], 'meaning')[0].get_signs() if
                          'block' in sign.name][0]
            table_name = [sign.name for sign in script.get_iner(self.world_model['ontable'], 'meaning')[0].get_signs() if
                          sign.name != block_name][0]
            new_x_y['objects'][block_name]['y'] = new_x_y['objects'][table_name]['y']
            new_x_y['objects'][block_name]['x'] = new_x_y['objects'][table_name]['x']
        region_map, cell_map, cell_location, near_loc, cell_coords_new = signs_markup(new_x_y, 'agent', cell_coords)

        sit_name = st.SIT_PREFIX + str(st.SIT_COUNTER)
        st.SIT_COUNTER+=1
        estimation = define_situation(sit_name + 'sp', cell_map, events, agent_state, self.world_model)
        estimation = update_situation(estimation, cell_map, self.world_model, fast_estimation)


        if flag:
            region = None
            for reg, cellz in cell_location.items():
                if 'cell-4' in cellz:
                    region = reg
            self.additions[3][iteration] = cell_map
            print("act: {0}, cell: {1}, dir: {2}, reg: {3}".format(script.sign.name, cell_coords_new['cell-4'],
                                                                   direction.name, region))
            return estimation, cell_coords_new, new_x_y, cell_location, near_loc, region_map, direction.name

        return estimation, cell_coords_new, new_x_y, cell_location, near_loc, region_map

    def _step_generating(self, active_pm, active_map, script, agent, iteration, prev_state, param):
        next_pm, cell_coords, parsed_map, cell_location, \
            near_loc, region_map, direction = self._state_prediction(active_pm, script, agent, iteration, param)
        prev_state.append(cell_coords['cell-4'])
        self.additions[0][iteration + 1] = cell_coords
        self.additions[1][iteration + 1] = parsed_map
        if self.change_map(active_map, cell_location):
            active_map = define_map(st.MAP_PREFIX + str(st.SIT_COUNTER), region_map, cell_location, near_loc, self.additions[2],
                                 self.world_model)
            print('map has been changed!')
        elif iteration > 0:
            if list(self.additions[3][iteration].values()) != list(self.additions[3][iteration - 1].values()):
                active_map = define_map(st.MAP_PREFIX + str(st.SIT_COUNTER), region_map, cell_location, near_loc,
                                     self.additions[2], self.world_model)
                print('block was moved!')

        return next_pm, active_map, prev_state, direction

    def _time_shift_backward(self, active_pm, script):
        next_pm = Sign(st.SIT_PREFIX + str(st.SIT_COUNTER))
        self.world_model[next_pm.name] = next_pm
        pm = next_pm.add_meaning()
        st.SIT_COUNTER += 1
        copied = {}
        for event in active_pm.cause:
            for es in script.effect:
                if event.resonate('meaning', es):
                    break
            else:
                pm.add_event(event.copy(pm, 'meaning', 'meaning', copied))
        for event in script.cause:
            pm.add_event(event.copy(pm, 'meaning', 'meaning', copied))
        return pm

    def _time_shift_forward(self, active_pm, script):
        next_pm = Sign(st.SIT_PREFIX + str(st.SIT_COUNTER))
        self.world_model[next_pm.name] = next_pm
        pm = next_pm.add_meaning()
        st.SIT_COUNTER += 1
        copied = {}
        for event in active_pm.cause:
            for es in script.cause:
                if event.resonate('meaning', es):
                    break
            else:
                pm.add_event(event.copy(pm, 'meaning', 'meaning', copied))
        for event in script.effect:
            pm.add_event(event.copy(pm, 'meaning', 'meaning', copied))
        return pm

    def _time_shift_forward_spat(self, active_pm, script):
        pm_events = []
        for event in active_pm.cause:
            for es in script.cause:
                if event.resonate('meaning', es):
                    break
            else:
                pm_events.append(event)
        for event in script.effect:
            pm_events.append(event)
        return pm_events

    def change_map(self, active_map, cell_location):
        pms = active_map.spread_down_activity('meaning', 4)
        pm_list = []
        contain_reg = None
        for location, cells in cell_location.items():
            if 'cell-4' in cells:
                contain_reg = location
        for iner in pms:
            iner_names = [s.sign.name for s in iner]
            if 'include' in iner_names:
                pm_list.append(iner[-1])
        for pm in pm_list:
            if pm.sign.name != 'cell-4':
                if pm.sign.name == contain_reg:
                    return False
        return True

    def get_stright(self, estimation, dir_sign):
        es = estimation.spread_down_activity('meaning', 4)
        grouped = {}
        for key, group in itertools.groupby(es, lambda x: x[1]):
            for pred in group:
                grouped.setdefault(key, []).append(pred[-1])
        stright_cell = None
        used_key = None
        for key, item in grouped.items():
            it_signs = {am.sign for am in item}
            if dir_sign in it_signs:
                stright_cell = [sign for sign in it_signs if
                                sign.name != 'cell-4' and sign != dir_sign and sign.name != "I"]
                if stright_cell:
                    stright_cell = stright_cell[0]
                    used_key = key
                    break
        for key, item in grouped.items():
            if key != used_key:
                it_signs_names = {am.sign.name for am in item}
                if stright_cell.name in it_signs_names:
                    if 'nothing' in it_signs_names:
                        return None
                    else:
                        items = [it for it in item if it.sign != stright_cell and it.sign.name != 'cell-4']
                        if items: return items

    @staticmethod
    def mix_pairs(replace_map):
        new_chain = {}
        elements = []
        merged_chains = []
        used_roles = []
        replace_map = list(replace_map.items())

        def get_role(obj, roles):
            for role in roles:
                if obj in role[1]:
                    return role

        for item in replace_map:
            elements.append(item[1])
        elements = list(itertools.product(*elements))
        clean_el = copy(elements)
        for element in clean_el:
            if not len(set(element)) == len(element):
                elements.remove(element)
        for element in elements:
            for obj in element:
                avalaible_roles = [x for x in replace_map if x not in used_roles]
                role = get_role(obj, avalaible_roles)
                if role:
                    used_roles.append(role)
                    new_chain[role[0]] = obj
            merged_chains.append(new_chain)
            new_chain = {}
            used_roles = []
        return merged_chains





# def _get_experience(agents):
#     actions = []
#     for agent in agents:
#         for connector in agent.out_meanings:
#             cm = connector.in_sign.meanings[connector.in_index]
#             if max([len(event.coincidences) for event in itertools.chain(cm.cause, cm.effect)]) > 1:
#                 if cm.is_causal():
#                     #for sign
#                     for pm in actions:
#                         if pm.resonate('meaning', cm):
#                             break
#                     else:
#                         actions.append(cm)
#
#     return actions








# def _check_experience(candidates, exp_actions):
#     actions = []
#     for candidate in candidates:
#         if candidate[3]:
#             if candidate[2] in exp_actions:
#                 actions.append(candidate)
#     if actions:
#         return actions
#     else:
#         return candidates








# def watch_matrix(event, base):
#     matrices = []
#     for connector in event.coincidences:
#         matrices.append(getattr(connector.out_sign, base+'s')[connector.out_index])
#
#     return matrices













