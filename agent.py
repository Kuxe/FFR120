import numpy as np
import time
import warnings
warnings.filterwarnings("ignore", category=np.VisibleDeprecationWarning)

# An agent which given a list of agents is capable of
# updating its position, velocity and acceleration
class Agent:

    position = np.array([0, 0]);
    velocity = np.array([1, 1]);
    acceleration = np.array([0, 0]);
    preferredVelocity = np.array([0, 0]); #v0_alpha * e_alpha, desired speed * direction of destination
    relaxation = 1.0
    
    def __init__(self, initialPosition, initialVelocity, preferredVelocity):
        self.position = initialPosition
        self.velocity = initialVelocity
        self.preferredVelocity = preferredVelocity
        
    # Behavioral force f_alpha(t) is the acceleration plus a fluctuation term
    def behavioral(self, agents, boundaries, attractors):
        return 1/self.relaxation * (self.preferredVelocity - self.velocity) + self.repulsiveEffects(boundaries) + self.repulsiveInteractions(agents) + self.attractionEffects(attractors) + self.fluctuation()
    
    def fluctuation(self):
        MAX = 1; MIN = -1
        return np.random.random(2) * (MAX-MIN) + MIN
    
    def repulsiveEffects(self, boundaries):
        sum = np.array([0.0, 0.0]);
        integer_position = np.array([round(self.position[0]), round(self.position[1])])
        integer_position = self.position
        x = np.array([i for j in range(len(boundaries[:,0])) \
                          for i in range(len(boundaries[0,:])) if boundaries[j,i] == True])
        y = np.array([j for j in range(len(boundaries[:,0])) \
                          for i in range(len(boundaries[0,:])) if boundaries[j,i] == True])
        for i in x:
            rab = integer_position - np.array([x[i],y[i]])
            sum += rab / np.dot(rab,rab)
        return sum
    
    def repulsiveInteractions(self, agents):
        # Almost Coulomb potential, Q = 1 temporary?
        sum = np.array([0.0, 0.0]);
        for agent in agents:
            if(agent != self):
                rab = self.position - agent.position
                sum += rab / np.dot(rab, rab)
        return sum

        # Alternative implementation, possibly easier to speed up with parallelization
        #rabs = [self.position - agent.position for agent in agents if agent != self]
        #return sum([rab / np.dot(rab, rab) for rab in rabs])

    
    def attractionEffects(self, attractors):
        # TODO: Implement according to paper
        return np.array([0, 0])
        
    def update(self, state):
        tmpPos = np.copy(self.position);
        self.acceleration = self.behavioral(state.agents, state.boundaryMap, state.attractors)
        self.velocity += self.acceleration
        self.position += self.velocity * state.dt
        state.totalDistanceTravelled += np.linalg.norm(self.position-tmpPos)
