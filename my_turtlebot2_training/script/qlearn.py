'''
Q-learning approach for different RL problems
as part of the basic series on reinforcement learning @
https://github.com/vmayoral/basic_reinforcement_learning

Inspired by https://gym.openai.com/evaluations/eval_kWknKOkPQ7izrixdhriurA

@author: Victor Mayoral Vilches <victor@erlerobotics.com>
'''
import random


class QLearn:
    def __init__(self, actions, epsilon, alpha, gamma):
        self.q = {}
        self.epsilon = epsilon  # exploration constant
        self.alpha = alpha      # discount constant
        self.gamma = gamma      # discount factor
        self.actions = actions

    def getQ(self, state, action):
        return self.q.get((state, action), 0.0)

    def learnQ(self, state, action, reward, value):
        # maxqnew = max([self.getQ(state2, a) for a in self.actions])
        # value = reward + gamma * maxqnew
        '''
        Q-learning:
            Q(s, a) += alpha * (reward(s,a) + max(Q(s') - Q(s,a))
            NO IT HAS TO BE >>
            Q(s, a) += alpha * (reward(s,a) + gamma*max(Q(s',a') - Q(s,a))
        '''
        oldv = self.q.get((state, action), None)  # old value
        if oldv is None:
            self.q[(state, action)] = reward
            # print("putting REWARD to q-table")
        else:
            self.q[(state, action)] = oldv + self.alpha * (value - oldv)  # q-learning equation
            # print("UPDATING q-table")

        # print("q = ", self.q)
        # print(self.q[(state, action)])
        # print("-------------------------------------------------")

    def chooseAction(self, state, return_q=False):
        # print("state = ", state)
        q = [self.getQ(state, a) for a in self.actions]  # get q-value of each action given a current state
        # print("q = ", q)
        maxQ = max(q)  # get the highest q-value
        # print("max q = ", maxQ)
        # print("-------------------------------------------------")

        if random.random() < self.epsilon:
            print("randomizing the action........")
            minQ = min(q)
            mag = max(abs(minQ), abs(maxQ))
            # add random values to all the actions, recalculate maxQ
            q = [q[i] + random.random() * mag - .5 * mag for i in range(len(self.actions))] 
            maxQ = max(q)
            # print("q = ", q)
        else:
            print("choosing the best action.......")
            # print("q = ", q)

        count = q.count(maxQ)
        # In case there're several state-action max values 
        # we select a random one among them
        if count > 1:
            best = [i for i in range(len(self.actions)) if q[i] == maxQ]
            i = random.choice(best)
        else:
            i = q.index(maxQ)

        action = self.actions[i]

        # print("i = ", i)
        # print("action = ", action)
        # print("q = ", q)
        # print("max q = ", max(q))
        # print("-------------------------------------------------")

        if return_q:  # if they want it, give it!
            return action, q
        return action

    def learn(self, state1, action1, reward, state2):
	print("UPDATING q-table")
        maxqnew = max([self.getQ(state2, a) for a in self.actions])  # max q value of newState-action pair (q*(s',a))
        self.learnQ(state1, action1, reward, reward + self.gamma*maxqnew)
