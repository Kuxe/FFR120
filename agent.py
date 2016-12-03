import numpy as np
import time
import warnings
warnings.filterwarnings("ignore", category=np.VisibleDeprecationWarning)

# An agent which given a list of agents is capable of
# updating its position, velocity and acceleration
class Agent:    
    def __init__(self, initialPosition, initialVelocity, preferredVelocity, agentGroup):
        self.position = initialPosition
        self.velocity = initialVelocity
        self.acceleration = np.array([0, 0])
        self.preferredVelocity = preferredVelocity
        self.relaxation = 0.01
        self.agentGroup = agentGroup
        self.inGoal = False
        
    # Behavioral force f_alpha(t) is the acceleration plus a fluctuation term
    def behavioral(self, agents, boundaries, attractors):
        return 1/self.relaxation * (self.preferredVelocity - self.velocity) + self.repulsiveEffects(boundaries) + self.repulsiveInteractions(agents) + self.attractionEffects(attractors) + self.fluctuation()
    
    def fluctuation(self):
        MAX = 1; MIN = -1
        return np.random.random(2) * (MAX-MIN) + MIN
    
    def repulsiveEffects(self, boundaries):
        sum = np.array([0.0, 0.0]);
        integer_position = np.array([int(round(self.position[0])), int(round(self.position[1]))])
        if integer_position[0]>0 and integer_position[0]<len(boundaries[0,:])-1:
            if integer_position[1]>0 and integer_position[1]<len(boundaries[:,0])-1:
                x = [i for j in range(integer_position[1]-1,integer_position[1]+2) \
                          for i in range(integer_position[0]-1,integer_position[0]+2) if boundaries[len(boundaries[:,0])-1-j, i] == True]
                y = [j for j in range(integer_position[1]-1,integer_position[1]+2) \
                          for i in range(integer_position[0]-1,integer_position[0]+2) if boundaries[len(boundaries[:,0])-1-j, i] == True]
                for i in range(0,len(x)):
                    rab = integer_position - np.array([x[i],y[i]])
                    if(np.dot(rab,rab) != 0):
                        sum += 100*rab / np.dot(rab,rab) 
        return sum
    
    def repulsiveInteractions(self, agents):
        # Almost Coulomb potential, Q = 1 temporary?
        sum = np.array([0.0, 0.0]);
        for agent in agents:
            if(agent != self):
                rab = self.position - agent.position
                sum += rab / np.dot(rab,rab)
        return sum

        # Alternative implementation, possibly easier to speed up with parallelization
        #rabs = [self.position - agent.position for agent in agents if agent != self]
        #return sum([rab / np.dot(rab, rab) for rab in rabs])

    
    def attractionEffects(self, attractors):
        # TODO: Implement according to paper
        return np.array([0, 0])
        
    def update(self, state):
        tmpPos = np.copy(self.position);
        self.acceleration = self.behavioral(state.agents, state.boundaryMap, state.attractors) * state.dt;
        self.velocity += self.acceleration * state.dt
        self.position += self.velocity * state.dt
        state.totalDistanceTravelled += np.linalg.norm(self.position-tmpPos)

        # Check if agents reached goal
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