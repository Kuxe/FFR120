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
    agents = []
    boundaryMap = None
    attractors = None

    useFixedTimeStep = False
    fixedTimeStep = None
    dt = None #Time-resolution of simulation

    totalDistanceTravelled = 0
    def __init__(self, numAgents, dt, boundaryMap):
        self.dt = dt
        self.boundaryMap = boundaryMap

        # TODO: Give agents some reasonable starting values (currently just randomize every member in range [-1, 1])

        
        # Agents should be split into two groups of equal size (could be interesting to look at different sizes too!)
        # with a preferred speed to opposite side of room.

        #pos, initvel, prefvel
        #self.agents = [Agent(np.random.random(2)*1, [0, 1], [0,1]) for i in range(int(numAgents))]
        #self.agents = self.agents.append([Agent(np.random.random(2)*1, [0, 1], [0,1]) for i in range(int(numAgents-(numAgents/2)))])

        # Initialize agents in boundaryMap, uniformly in two rectangles

        wallXStart = 0
        wallXEnd = 19
        wallYStart = 0
        wallYEnd = 19
        margin = 1
        side = 1;

        numAgents1 = int(numAgents/2);
        agentsXs = np.random.uniform(wallXStart+margin, wallXStart+margin+side, numAgents1)
        agentsYs = np.random.uniform(wallYStart+margin, wallYEnd-margin, numAgents1)
        for i in range(numAgents1):
            self.agents.append(Agent(np.array([agentsXs[i], agentsYs[i]]), np.array([0.0, 1.0]), np.array([1.0, 0.0])))

        numAgents2 = numAgents-int((numAgents/2))
        agentsXs = np.random.uniform(wallXEnd-margin-side, wallXEnd-margin, numAgents2)
        agentsYs = np.random.uniform(wallYStart+margin, wallYEnd-margin, numAgents2)
        for i in range(numAgents2):
            self.agents.append(Agent(np.array([agentsXs[i], agentsYs[i]]), np.array([0.0, -1.0]), np.array([-1.0, 0.0])))

        if(dt != 0.0):
            self.useFixedTimeStep = True
            self.fixedTimeStep = dt