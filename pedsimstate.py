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
    dt = None

    totalDistanceTravelled = 0
    def __init__(self, numAgents, dt, boundaryMap):
        self.dt = dt
        self.boundaryMap = boundaryMap

        # TODO: Give agents some reasonable starting values (currently just randomize every member in range [-1, 1])
        self.agents = [Agent(np.random.random(2)*2-1, (np.random.random(2)*2-1), (np.random.random(2)*2-1)) for i in range(numAgents)]
        if(dt != 0.0):
            self.useFixedTimeStep = True
            self.fixedTimeStep = dt