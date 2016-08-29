import random
from environment import Agent, Environment
from planner import RoutePlanner
from simulator import Simulator
import numpy as np
import pandas as pd

class LearningAgent(Agent):
    """An agent that learns to drive in the smartcab world."""

    def __init__(self, env):
        super(LearningAgent, self).__init__(env)  # sets self.env = env, state = None, next_waypoint = None, and a default color
        self.color = 'red'  # override color
        self.planner = RoutePlanner(self.env, self)  # simple route planner to get next_waypoint
        # TODO: Initialize any additional variables here
        self.gamma = 0.8
        self.q_table = {}
        self.prev_state = []
        self.prev_reward = []
        self.prev_action = []
        self.count = 0
        self.epsilon = 1.02
        self.alpha = 0.2

    def reset(self, destination=None):
        # final_q_table = pd.DataFrame.from_dict(self.q_table, orient='index')
        # print final_q_table
        if self.epsilon >= 0.02:
            self.epsilon = self.epsilon - 0.02
        print "epsilon =" + str(self.epsilon)
        self.planner.route_to(destination)
        # TODO: Prepare for a new trip; reset any variables here, if required

    def update(self, t):
        # Gather inputs
        self.next_waypoint = self.planner.next_waypoint()  # from route planner, also displayed by simulator
        inputs = self.env.sense(self)
        deadline = self.env.get_deadline(self)

        # Update state (choose traffic_light, oncoming_straight, oncoming_right, oncoming_left)
        next_point = self.next_waypoint
        self.state = (
        ('traffic_light', inputs['light']),
        ('in_oncoming', inputs['oncoming']),
        ('in_right', inputs['right']),
        ('in_left', inputs['left']),
        ('next_point', next_point))
        self.prev_state.append(self.state)

        # Select action (exploration vs exploitation)
        expl_vs_expl = np.random.choice(['exploration', 'exploitation'], p=[self.epsilon, 1-self.epsilon])
        if expl_vs_expl == 'exploitation' and self.state in self.q_table:
            print "Agent is doing exploitation!"
            action = action = max(self.q_table[self.state], key=self.q_table[self.state].get)
        else:
            print "Agent is doing exploration!"
            action = random.choice([None, 'right', 'left', 'forward'])
        reward = self.env.act(self, action)
        self.prev_action.append(action)
        self.prev_reward.append(reward)
        # Update Q-Learning table
        if self.count >= 1:
            if self.prev_state[self.count-1] in self.q_table:
                self.q_table[self.prev_state[self.count-1]][self.prev_action[self.count-1]] = \
                (1-self.alpha)*self.q_table.get(self.prev_state[self.count-1], {}).get(self.prev_action[self.count-1], 0) + \
                self.alpha*(self.prev_reward[self.count-1] + self.gamma * self.q_table.get(self.prev_state[self.count], {}).get(self.prev_action[self.count], 0))
            else:
                self.q_table[self.prev_state[self.count-1]] = {}
                self.q_table[self.prev_state[self.count-1]][self.prev_action[self.count-1]] = \
                (1-self.alpha)*self.q_table.get(self.prev_state[self.count-1], {}).get(self.prev_action[self.count-1], 0) + \
                self.alpha*(self.prev_reward[self.count-1] + self.gamma * self.q_table.get(self.prev_state[self.count], {}).get(self.prev_action[self.count], 0))
        self.count = self.count + 1

        print "LearningAgent.update(): deadline = {}, inputs = {}, action = {}, reward = {}".format(deadline, inputs, action, reward)  # [debug]


def run():
    """Run the agent for a finite number of trials."""

    # Set up environment and agent
    e = Environment()  # create environment (also adds some dummy traffic)
    a = e.create_agent(LearningAgent)  # create agent
    e.set_primary_agent(a, enforce_deadline=True)  # specify agent to track

    # Now simulate it
    sim = Simulator(e, update_delay=0.5, display=True)  # create simulator (uses pygame when display=True, if available)

    sim.run(n_trials=50)  # run for a specified number of trials


if __name__ == '__main__':
    run()
