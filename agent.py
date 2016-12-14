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
        self.acceleration = np.array([0.0, 0.0])
        self.relaxation = 0.02
        self.agentGroup = agentGroup
        self.inGoal = False
        self.cumVelocity = np.array([0.0, 0.0])
        self.cumSpeed = 0
        self.cumSpeedPreferred = 0
        self.cumSpeedSquared = 0
        self.preferredSpeed = np.linalg.norm(self.preferredVelocity)
        
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
        WALL_SCALAR = 1.5
        
        if distanceToLower < distanceToHigher:
            return np.array([0.0, 1.0]) * (np.exp(WALL_SCALAR/distanceToLower) - 1.0)
        return np.array([0.0, -1.0]) * (np.exp(WALL_SCALAR/distanceToHigher) - 1.0)

    
    def repulsiveInteractions(self, agents):
        # Almost Coulomb potential, Q = 1 temporary?
        sum1 = np.array([0.0, 0.0]);
        sum2 = np.array([0.0, 0.0]);
        rmin1 = 2.0**2 #Because comparison done with squared euclidean distance as opposed to euclidean distance (dot faster than norm)
        rmin2 = 1.5**2
        COULUMB_SCALAR1 = 10.0
        COULUMB_SCALAR2 = 15.0
        Y_MAGNIFICATION1 = 1.5
        Y_MAGNIFICATION2 = 2.5
        

        for agent in agents:
            if(agent != self):
                rab = self.position - agent.position
                rabdot = np.dot(rab, rab)
                sameGroup = self.agentGroup == agent.agentGroup
                if sameGroup and (rabdot < rmin1):
                    sum1 += rab/rabdot
                if not sameGroup and (rabdot < rmin2):
                    sum2 += rab/rabdot
        return np.multiply(COULUMB_SCALAR1*sum1, np.array([1.0, Y_MAGNIFICATION1])) + np.multiply(COULUMB_SCALAR2*sum2, np.array([1.0, Y_MAGNIFICATION2]))

        # DO NOT REMOVE! THIS IS _MUCH_FASTER FOR NUM_AGENTS >= 200
        #g = self.agentGroup != np.matrix([agent.agentGroup for agent in agents if agent != self]).T
        #rab = -np.matrix([agent.position for agent in agents if agent != self]) + self.position;
        #rabdot = np.square(rab).sum(1);
        #COULUMB_SCALAR1 *= (1.0-g)
        #COULUMB_SCALAR1[rabdot > rmin1] = 0.0
        #COULUMB_SCALAR2 *= g
        #COULUMB_SCALAR2[rabdot > rmin2] = 0.0
        #return ((rab / rabdot).T * (COULUMB_SCALAR2 + COULUMB_SCALAR1)).A1
        
    def update(self, state, pedsim):
        self.acceleration = self.behavioral(state.agents, state.boundaryMap, state.attractors)
        self.velocity += self.acceleration * state.dt
        if(self.agentGroup == 0 and (self.velocity[0] < 0)):
            self.velocity[0] = 0.01
        if(self.agentGroup == 1 and (self.velocity[0] > 0)):
            self.velocity[0] = -0.01

        # Cap magnitude of vector to 12.4m/s (Usain Bolt 2009 Berlin)
        velocityMagnitude = np.linalg.norm(self.velocity)
        MAX_SPEED = 10.0;
        if(velocityMagnitude > MAX_SPEED):
            self.velocity = self.velocity / velocityMagnitude * MAX_SPEED;

        self.position += self.velocity * state.dt

        # Confine agents within boundary
        WALL_WIDTH = 0.1;
        self.position[1] = np.clip(self.position[1], 0+WALL_WIDTH, np.size(state.boundaryMap, 0)-1-WALL_WIDTH)

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
            else:
                self.inGoal = self.position[0] < state.goalLineLeft
            return self.inGoal;
        return 0