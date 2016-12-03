import numpy as np
import time
import warnings
warnings.filterwarnings("ignore", category=np.VisibleDeprecationWarning)
from agent import Agent

# PedsimState holds whatever data that defines an instance of a simulation
# So if we want to run Pedsim for different set of parameters, it boils down to
# instanciating different PedsimStates and feeding each PedsimState to Pedsim.update(PedsimState)
# Any PedsimState can be fed to PedsimVisualizer via PedsimVisualizer.visualize(PedsimState)
class PedsimState:
    def __init__(self, numAgents, dt, boundaryMap, mean, variance):

        self.boundaryMap = None
        self.attractors = None

        self.useFixedTimeStep = False
        self.fixedTimeStep = None
        self.dt = None #Time-resolution of simulation
        self.runningTimePerStep = None #How many milliseconds spend per call to simulate

        self.goalLeft = None
        self.goalRight = None

        self.numAgents = 0
        self.numAgentsInGoal = 0
        self.goalLineLeft = None
        self.goalLineRight = None

        self.mean = -1;
        self.variance = -1

        self.totalDistanceTravelled = 0

        #Variables to save data from..
        #TODO: Get better variablenames
        self.time = [0]
        self.efficiencyLevels = []

        self.agents = []
        self.dt = dt
        self.boundaryMap = boundaryMap
        self.numAgents = numAgents
        self.mean = mean
        self.variance = variance

        # Agents should be split into two groups of equal size (could be interesting to look at different sizes too!)
        # with a preferred speed to opposite side of room. Initialize agents in boundaryMap, uniformly in two rectangles.
        bmshape = np.shape(boundaryMap)
        wallXStart = 0
        wallXEnd = bmshape[1]-1
        wallYStart = 0
        wallYEnd = bmshape[0]-1
        margin = 1
        side = 1

        self.goalLineRight = wallXEnd-margin-side
        self.goalLineLeft = wallXStart+margin+side

        numAgents1 = int(self.numAgents/2);
        agentsXs = np.random.uniform(wallXStart+margin, wallXStart+margin+side, numAgents1)
        agentsYs = np.random.uniform(wallYStart+margin, wallYEnd-margin, numAgents1)
        for i in range(numAgents1):
            print("len:%i" % len(self.agents))
            self.agents.append(Agent(np.array([agentsXs[i], agentsYs[i]]), np.array([1.0, 0.0]), np.array([1.0, 0.0]), 0))

        numAgents2 = self.numAgents-int((self.numAgents/2))
        agentsXs = np.random.uniform(wallXEnd-margin-side, wallXEnd-margin, numAgents2)
        agentsYs = np.random.uniform(wallYStart+margin, wallYEnd-margin, numAgents2)
        for i in range(numAgents2):
            print("len:%i" % len(self.agents))
            self.agents.append(Agent(np.array([agentsXs[i], agentsYs[i]]), np.array([-1.0, 0.0]), np.array([-1.0, 0.0]), 1))

        print("Initialized with %i agents, actually %i, where group #1 is %i and group #2 is %i" % (self.numAgents, len(self.agents), numAgents1, numAgents2))

        if(dt != 0.0):
            self.useFixedTimeStep = True
            self.fixedTimeStep = dt