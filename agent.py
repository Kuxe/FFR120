import numpy as np
import time
import warnings
warnings.filterwarnings("ignore", category=np.VisibleDeprecationWarning)

# An agent which given a list of agents is capable of
# updating its position, velocity and acceleration
class Agent:    
    def __init__(self, initialPosition, preferredVelocity, agentGroup):
        self.position = initialPosition
        self.position0 = np.copy(self.position)
        self.preferredVelocity = preferredVelocity
        self.velocity = np.copy(self.preferredVelocity)
        self.acceleration = np.array([0, 0])
        self.relaxation = 0.02
        self.agentGroup = agentGroup
        self.inGoal = False
        
    # Behavioral force f_alpha(t) is the acceleration plus a fluctuation term
    def behavioral(self, agents, boundaries, attractors):
        return (self.preferredVelocity - self.velocity)/self.relaxation + \
        self.repulsiveEffects(boundaries) + self.repulsiveInteractions(agents) + \
        self.fluctuation()
    
    def fluctuation(self):
        MAX = 1; MIN = -1
        return np.random.random(2) * (MAX-MIN) + MIN

    def repulsiveEffects(self, boundaries):
        s = np.shape(boundaries)
        upperBound = s[0]-1
        lowerBound = 0
        distanceToLower = abs(self.position[1] - lowerBound)
        distanceToHigher = abs(self.position[1] - upperBound)

        if distanceToLower < distanceToHigher:
            return np.array([0.0, 1.0]) * (np.exp(0.7/distanceToLower) - 1)
        return np.array([0.0, -1.0]) * (np.exp(0.7/distanceToHigher) - 1)

    
    def repulsiveInteractions(self, agents):
        # Almost Coulomb potential, Q = 1 temporary?
        sum1 = np.array([0.0, 0.0]);
        sum2 = np.array([0.0, 0.0]);
        sum = np.array([0.0, 0.0]);
        rmin1 = 1**2 #Because comparison done with squared euclidean distance as opposed to euclidean distance (dot faster than norm)
        rmin2 = 1.5**2
        COULUMB_SCALAR1 = 5
        COULUMB_SCALAR2 = 10
        for agent in agents:
            if(agent != self):
                rab = self.position - agent.position
                if self.agentGroup == agent.agentGroup:
                    if np.dot(rab, rab) < rmin1:
                        sum1 += COULUMB_SCALAR1*(rab)/np.dot(rab,rab)
                        #rmin1 = np.linalg.norm(rab)
                if self.agentGroup != agent.agentGroup:
                    if np.dot(rab, rab) < rmin2:
                        sum2 += COULUMB_SCALAR2*(rab)/np.dot(rab,rab)
                        #rmin2 = np.linalg.norm(rab)
        return sum1 + sum2
        
    def update(self, state, pedsim):
        tmpPos = np.copy(self.position)
        self.acceleration = self.behavioral(state.agents, state.boundaryMap, state.attractors) * state.dt
        self.velocity += self.acceleration * state.dt
        if(self.agentGroup == 0):
            if(self.velocity[0] < 0):
                self.velocity[0] = 0
        if(self.agentGroup == 1):
            if(self.velocity[0] > 0):
                self.velocity[0] = 0
        self.position += self.velocity * state.dt

        # Check if agents reached goal
        if(pedsim.continuous):
            state.numAgentsInGoal += self.goal(state)
            if(self.inGoal):
                self.position[0] = self.position0[0]
                #self.velocity = self.preferredVelocity
                #self.position[1] = np.random.uniform(1,6)
                self.inGoal = False
        else:
            state.numAgentsInGoal += self.goal(state)
        

    # Method that returns 1 (true) if agent is at other side of goal line, otherwise 0 (false)
    def goal(self, state):
        if(not self.inGoal):
            if(self.agentGroup == 0):
                self.inGoal = self.position[0] > state.goalLineRight
                return self.inGoal
            else:
                self.inGoal = self.position[0] < state.goalLineLeft
                return self.inGoal
        return 0