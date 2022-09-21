import sys
import time
from constants import *
from environment import *
from state import State
"""
solution.py

This file is a template you should use to implement your solution.

You should implement each section below which contains a TODO comment.

COMP3702 2022 Assignment 2 Support Code

Last updated by njc 08/09/22
"""


class Solver:

    def __init__(self, environment: Environment):
        self.environment = environment
        #
        # TODO: Define any class instance variables you require (e.g. dictionary mapping state to VI value) here.
        #

        #set containing all possible states hexbot environment can be in         
        self.states = None
        #dict mapping state -> value 
        self.values = None
        #dict mapping state -> action
        self.policy = None
        self.epsilon = self.environment.epsilon
        self.gamma = self.environment.gamma
        self.hasConverged = False
        #drift and double move probabilities
        self.drift_cw_probs = self.environment.drift_cw_probs
        self.drift_ccw_probs = self.environment.drift_ccw_probs
        self.double_move_probs = self.environment.double_move_probs

    # === Value Iteration ==============================================================================================

    def vi_initialise(self):
        """
        Initialise any variables required before the start of Value Iteration.
        """
        #
        # TODO: Implement any initialisation for Value Iteration (e.g. building a list of states) here. You should not
        #  perform value iteration in this method.
        #
        # In order to ensure compatibility with tester, you should avoid adding additional arguments to this function.
        #

        self.states = self.initialise_states()
        #initialise all initial values to 0
        self.values = {s: 0 for s in self.states}
        self.policy = {s: REVERSE for s in self.states}

    def vi_is_converged(self):
        """
        Check if Value Iteration has reached convergence.
        :return: True if converged, False otherwise
        """
        #
        # TODO: Implement code to check if Value Iteration has reached convergence here.
        #
        # In order to ensure compatibility with tester, you should avoid adding additional arguments to this function.
        #
        return self.hasConverged

    def vi_iteration(self):
        """
        Perform a single iteration of Value Iteration (i.e. loop over the state space once).
        """
        #
        # TODO: Implement code to perform a single iteration of Value Iteration here.
        #
        # In order to ensure compatibility with tester, you should avoid adding additional arguments to this function.
        #
        newValues = dict()
        newPolicy = dict()
        for s in self.states:
            if self.environment.is_solved(s):
                newValues[s] = 0
                continue

            actionValues = dict()
            for a in ROBOT_ACTIONS:
                totalValue = 0
                # print(self.get_stoch_actions(a))
                #loop through each of the six possible combinations of actions
                for stochActions, p in self.get_stoch_actions(a):
                    sNext = s
                    minReward = 0
                    #loop through the list of actions we peform to find the one 
                    #with the minimum reward
                    for miniAction in stochActions:
                        reward, sNext = self.environment.apply_dynamics(s, miniAction)
                        minReward = min(reward, minReward)
                    #set positive reward if in goal state
                    totalValue += p * (minReward + (self.gamma * self.values[sNext]))
                actionValues[a] = totalValue
            # break
            newValues[s] = max(actionValues.values())
            newPolicy[s] = self.max_action_value(actionValues)

        #check for convergence
        differences = [abs(self.values[s] - newValues[s]) for s in self.states]            
        maxDiff = max(differences)
        if maxDiff < self.epsilon:
            self.hasConverged = True

        #update values and policy
        self.values = newValues
        self.policy = newPolicy

    def vi_plan_offline(self):
        """
        Plan using Value Iteration.
        """
        # !!! In order to ensure compatibility with tester, you should not modify this method !!!
        self.vi_initialise()
        while not self.vi_is_converged():
            self.vi_iteration()

    def vi_get_state_value(self, state: State):
        """
        Retrieve V(s) for the given state.
        :param state: the current state
        :return: V(s)
        """
        #
        # TODO: Implement code to return the value V(s) for the given state (based on your stored VI values) here. If a
        #  value for V(s) has not yet been computed, this function should return 0.
        #
        # In order to ensure compatibility with tester, you should avoid adding additional arguments to this function.
        #
        return self.values.get(state, 0)

    def vi_select_action(self, state: State):
        """
        Retrieve the optimal action for the given state (based on values computed by Value Iteration).
        :param state: the current state
        :return: optimal action for the given state (element of ROBOT_ACTIONS)
        """
        #
        # TODO: Implement code to return the optimal action for the given state (based on your stored VI values) here.
        #
        # In order to ensure compatibility with tester, you should avoid adding additional arguments to this function.
        #
        return self.policy[state]

    # === Policy Iteration =============================================================================================

    def pi_initialise(self):
        """
        Initialise any variables required before the start of Policy Iteration.
        """
        #
        # TODO: Implement any initialisation for Policy Iteration (e.g. building a list of states) here. You should not
        #  perform policy iteration in this method. You should assume an initial policy of always move FORWARDS.
        #
        # In order to ensure compatibility with tester, you should avoid adding additional arguments to this function.
        #
        pass

    def pi_is_converged(self):
        """
        Check if Policy Iteration has reached convergence.
        :return: True if converged, False otherwise
        """
        #
        # TODO: Implement code to check if Policy Iteration has reached convergence here.
        #
        # In order to ensure compatibility with tester, you should avoid adding additional arguments to this function.
        #
        pass

    def pi_iteration(self):
        """
        Perform a single iteration of Policy Iteration (i.e. perform one step of policy evaluation and one step of
        policy improvement).
        """
        #
        # TODO: Implement code to perform a single iteration of Policy Iteration (evaluation + improvement) here.
        #
        # In order to ensure compatibility with tester, you should avoid adding additional arguments to this function.
        #
        pass

    def pi_plan_offline(self):
        """
        Plan using Policy Iteration.
        """
        # !!! In order to ensure compatibility with tester, you should not modify this method !!!
        self.pi_initialise()
        while not self.pi_is_converged():
            self.pi_iteration()

    def pi_select_action(self, state: State):
        """
        Retrieve the optimal action for the given state (based on values computed by Value Iteration).
        :param state: the current state
        :return: optimal action for the given state (element of ROBOT_ACTIONS)
        """
        #
        # TODO: Implement code to return the optimal action for the given state (based on your stored PI policy) here.
        #
        # In order to ensure compatibility with tester, you should avoid adding additional arguments to this function.
        #
        pass

    # === Helper Methods ===============================================================================================
    #
    #
    # TODO: Add any additional methods here
    #
    #

    def get_stoch_actions(self, action):
        """
        Returns a dictionary mapping:
            (list of actions) -> probability

        Each "list of actions" corresponds to one of the following 6 possible 
        move orders given the specific action:
            move
            move, move
            drift cw, move
            drift cw, move, move
            drift ccw, move
            drift ccw, move, move
        
        """
        double_move = self.double_move_probs[action]
        drift_left = self.drift_cw_probs[action]
        drift_right = self.drift_ccw_probs[action]

        #calculate probabilities of each distinct action order
        stoch_actions = []
        stoch_actions.append(([action, action], double_move * (1 - drift_left - drift_right)))
        stoch_actions.append(([SPIN_LEFT, action], drift_left * (1 - double_move)))
        stoch_actions.append(([SPIN_LEFT, action, action], drift_left * double_move))
        stoch_actions.append(([SPIN_RIGHT, action], drift_right * (1 - double_move)))
        stoch_actions.append(([SPIN_RIGHT, action, action], drift_right * double_move))
        stoch_actions.append(([action], 1 - sum(stoch[1] for stoch in stoch_actions)))

        return stoch_actions

    def initialise_states(self):
        """
        Performs DFS from the initial state to generate all possible states in 
        the environment
        """
        states = set()
        frontier = set()
        initState = self.environment.get_init_state()
        frontier.add(initState)

        while len(frontier) > 0:
            currState = frontier.pop()
            states.add(currState)

            for action in ROBOT_ACTIONS:
                _, nextState = self.environment.apply_dynamics(currState, action)
                if nextState not in states:
                    frontier.add(nextState)

        return states 
    
    
    def max_action_value(self, d):
        """
        Returns the action corresponding to the maximum value in the given 
        actionValues dictionary

        params:
            actionValues: dict[action, value]
        
        Returns:
            max value action
        """
        max_value = max(d.values())
        for k, v in d.items():
            if v == max_value:
                return k




