import random
import itertools
from collections import OrderedDict


class TrafficLight:
    """
    A traffic light that switches periodically.
    """

    VALID_STATES = ['NS', 'EW']

    def __init__(self, state=None, period=None):
        self.state = state if state is not None else random.choice(self.VALID_STATES)
        self.period = period if period is not None else random.choice([2, 3, 4, 5])
        self.last_updated = 0

    def reset(self):
        self.last_updated = 0

    def update(self, t):
        if t - self.last_updated >= self.period:
            self.state = 'NS' if  self.state == 'EW' else 'EW'
            self.last_updated = t


class Environment:
    """
    Environment within which all agents operate.
    """

    VALID_ACTIONS = [None, 'forward', 'left', 'right']
    VALID_INPUTS = {
        'light': TrafficLight.VALID_STATES,
        'oncoming': VALID_ACTIONS,
        'left': VALID_ACTIONS,
        'right': VALID_ACTIONS}
    VALID_HEADINGS = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # ENWS
    # VALID_HEADINGS = ['E', 'N', 'W', 'S']
    HARD_TIME_LIMIT = -100  #(to avoid deadlock)

    def __init__(self, dummy_agent_num=3):
        self.dummy_agent_num = dummy_agent_num

        # Initialize simulation variables
        self.done = False
        self.t = 0
        self.agent_states = OrderedDict()
        self.status_text = ""

        # Road network
        self.grid_size = (6, 8)
        self.bounds = (1, 1, self.grid_size[0], self.grid_size[1])
        self.block_size = 100
        self.roads = []
        self.success_count = 0
        self.intersections = OrderedDict()
        for row in range(self.bounds[0], self.bounds[2] + 1):
            for col in range(self.bounds[1], self.bounds[3] + 1):
                self.intersections[(row, col)] = TrafficLight()  # a traffic light at each intersection

        for intersection_a, intersection_b in itertools.product(self.intersections, self.intersections):
            if intersection_a == intersection_b:
                continue
            row_a, col_a = intersection_a
            row_b, col_b = intersection_b
            if (abs(row_a - row_b) + abs(col_a - col_b)) == 1:  # L1 distance = 1
                self.roads.append((intersection_a, intersection_b))

        # Dummy agents
        for _ in range(self.dummy_agent_num):
            self.create_agent(DummyAgent)

        # Primary agent and associated parameters
        self.primary_agent = None  # to be set explicitly
        self.enforce_deadline = False

    def create_agent(self, agent_class, *args, **kwargs):
        agent = agent_class(self, *args, **kwargs)
        self.agent_states[agent] = {'location': random.choice(list(self.intersections.keys())),
                                    'heading': (-1, 0)}
        return agent

    def set_primary_agent(self, agent, enforce_deadline=False):
        self.primary_agent = agent
        self.enforce_deadline = enforce_deadline

    def reset(self):
        self.done = False
        self.t = 0

        # Reset traffic lights
        for traffic_light in self.intersections.values():
            traffic_light.reset()

        # Pick a start and a destination
        smart_agent_start = random.choice(list(self.intersections.keys()))
        smart_agent_destination = random.choice(list(self.intersections.keys()))

        # Ensure starting location and destination are not too close
        while self.compute_dist(smart_agent_start, smart_agent_destination) < 4:
            smart_agent_start = random.choice(list(self.intersections.keys()))
            smart_agent_destination = random.choice(list(self.intersections.keys()))

        start_heading = random.choice(self.VALID_HEADINGS)
        deadline = self.compute_dist(smart_agent_start, smart_agent_destination) * 5
        print("Environment.reset(): Trial set up with start = {}, destination = {}, deadline = {}".\
              format(smart_agent_start, smart_agent_destination, deadline))

        # Initialize agent(s)
        for agent in self.agent_states.keys():
            if agent is self.primary_agent:
                self.agent_states[agent] = {
                    'location': smart_agent_start,
                    'heading': start_heading,
                    'destination': smart_agent_destination,
                    'deadline': deadline
                }
                agent.reset(destination=smart_agent_destination)
            else:
                self.agent_states[agent] = {
                    'location': random.choice(list(self.intersections.keys())),
                    'heading': random.choice(self.VALID_HEADINGS),
                    'destination': None,
                    'deadline': None}

                agent.reset()

    def step(self):
        #print("Environment.step(): t = {}".format(self.t))  # [debug]

        # Update traffic lights
        for intersection, traffic_light in self.intersections.items():
            traffic_light.update(self.t)

        # Update agents
        for agent in self.agent_states.keys():
            agent.update(self.t)

        if self.done:
            return  # primary agent might have reached destination

        if self.primary_agent is not None:
            agent_deadline = self.agent_states[self.primary_agent]['deadline']
            if agent_deadline <= self.HARD_TIME_LIMIT:
                self.done = True
                print("Environment.step(): Primary agent hit hard time limit ({})! Trial aborted.".\
                      format(self.HARD_TIME_LIMIT))
            elif self.enforce_deadline and agent_deadline <= 0:
                self.done = True
                print("Primary agent ran out of time! Trial aborted.")
                print("There has been {} successful trials!".format(self.success_count))
            self.agent_states[self.primary_agent]['deadline'] = agent_deadline - 1

        self.t += 1

    def sense(self, agent):
        assert agent in self.agent_states, "Unknown agent!"

        state = self.agent_states[agent]
        location = state['location']
        heading = state['heading']
        light = 'red'
        if self.intersections[location].state == 'NS' and heading[0] or\
                self.intersections[location].state == 'EW' and heading[1]:
            light = 'green'
        # Populate oncoming, left, right
        oncoming = None
        left = None
        right = None
        for other_agent, other_state in self.agent_states.items():
            if agent == other_agent or\
                    location != other_state['location'] or\
                    heading == other_state['heading']:
                continue

            other_heading = other_agent.get_next_waypoint()
            if (heading[0] * other_state['heading'][0] + heading[1] * other_state['heading'][1]) == -1:
                if oncoming != 'left':  # we don't want to override oncoming == 'left'
                    oncoming = other_heading
            elif heading[1] == other_state['heading'][0] and -heading[0] == other_state['heading'][1]:
                if right != 'forward' and right != 'left':  # we don't want to override right == 'forward or 'left'
                    right = other_heading
            else:
                if left != 'forward':  # we don't want to override left == 'forward'
                    left = other_heading

        return {'light': light, 'oncoming': oncoming, 'left': left, 'right': right}

    def get_deadline(self, agent):
        return self.agent_states[agent]['deadline']

    def act(self, agent, action):
        assert agent in self.agent_states, "Unknown agent!"
        assert action in self.VALID_ACTIONS, "Invalid action!"

        state = self.agent_states[agent]
        location = state['location']
        heading = state['heading']
        inputs = self.sense(agent)
        light = 'red'
        if self.intersections[location].state == 'EW' and heading[1] or\
                not self.intersections[location].state == 'NS' and heading[0]:
            light = 'green'

        # Move agent if within bounds and obeys traffic rules
        reward = 0  # reward/penalty
        move_okay = True
        if action == 'forward':
            if light != 'green':
                move_okay = False
        elif action == 'left':
            if light == 'green' and (inputs['oncoming'] is None or inputs['oncoming'] == 'left'):
                heading = (-heading[1], heading[0])
            else:
                move_okay = False
        elif action == 'right':
            if light == 'green' or inputs['left'] != 'forward':
                heading = (heading[1], -heading[0])
            else:
                move_okay = False

        if move_okay:
            if action is not None:
                # Valid non-null move
                location = ((location[0] + heading[0] - self.bounds[0]) % (self.bounds[2] - self.bounds[0] + 1) + self.bounds[0],
                            (location[1] + heading[1] - self.bounds[1]) % (self.bounds[3] - self.bounds[1] + 1) + self.bounds[1])  # wrap-around
                #if self.bounds[0] <= location[0] <= self.bounds[2] and self.bounds[1] <= location[1] <= self.bounds[3]:  # bounded
                state['location'] = location
                state['heading'] = heading
                reward = 2.0 if action == agent.get_next_waypoint() else -0.5  # valid, but is it correct? (as per waypoint)
            else:
                # Valid null move
                reward = 0.0
        else:
            # Invalid move
            reward = -1.0

        if agent is self.primary_agent:
            if state['location'] == state['destination']:
                if state['deadline'] >= 0:
                    reward += 10  # bonus
                self.done = True
                print("Environment.act(): Primary agent has reached destination!")  # [debug]
                self.success_count = self.success_count + 1
                print("There has been {} successful trials!".format(self.success_count))
            self.status_text = "state: {}\naction: {}\nreward: {}".format(agent.get_state(), action, reward)
            #print "Environment.act() [POST]: location: {}, heading: {}, action: {}, reward: {}".format(location, heading, action, reward)  # [debug]

        return reward

    def compute_dist(self, a, b):
        """
        L1 distance between two points.
        """
        return abs(b[0] - a[0]) + abs(b[1] - a[1])


class Agent:
    """
    Base class for all agents.
    """

    def __init__(self, env):
        self.env = env
        self.state = None
        self.next_waypoint = None
        self.color = 'cyan'

    def reset(self, destination=None):
        pass

    def update(self, t):
        pass

    def get_state(self):
        return self.state

    def get_next_waypoint(self):
        return self.next_waypoint


class DummyAgent(Agent):
    color_choices = ['blue', 'cyan', 'magenta', 'orange']

    def __init__(self, env):
        super(DummyAgent, self).__init__(env)  # sets self.env = env, state = None, next_waypoint = None, and a default color
        self.next_waypoint = random.choice(Environment.VALID_ACTIONS[1:])
        self.color = random.choice(self.color_choices)

    def update(self, t):
        inputs = self.env.sense(self)

        action_okay = True
        if self.next_waypoint == 'right':
            if inputs['light'] == 'red' and inputs['left'] == 'forward':
                action_okay = False
        elif self.next_waypoint == 'forward':
            if inputs['light'] == 'red':
                action_okay = False
        elif self.next_waypoint == 'left':
            if inputs['light'] == 'red' or (inputs['oncoming'] == 'forward' or inputs['oncoming'] == 'right'):
                action_okay = False

        action = None
        if action_okay:
            action = self.next_waypoint
            self.next_waypoint = random.choice(Environment.VALID_ACTIONS[1:])
        reward = self.env.act(self, action)
        # print("DummyAgent.update(): t = {}, inputs = {}, action = {}, reward = {}".format(t, inputs, action, reward))  # [debug]
        # print("DummyAgent.update(): next_waypoint = {}".format(self.next_waypoint))  # [debug]
