import numpy as np
import time
import argparse
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import warnings
warnings.filterwarnings("ignore", category=np.VisibleDeprecationWarning)
from pedsimvisualizer import PedsimVisualizer
from pedsimstate import PedsimState
from Boundarymap import *
import pickle

# The Pedestrian simulator Pedsim have PedsimState(s) which Pedsim can update
# and a visualizer which can visualize the state
class Pedsim:   
    def __init__(self, numAgents, plotdirections, plotaccelerations, plotRefreshRate, dt, enablePlotting, continuous, boundaryMap):
        self.visualizer = None

        # If enablePlotting=False, do not plot at all. Dont even create a window.
        # This is usefulf for running several simulations in parallel without wasting memory on GUI.
        self.enablePlotting = None

        self.dt = None
        self.boundaryMap = None
        self.numAgents = None
        self.continuous = None
        self.enablePlotting = enablePlotting
        self.numAgents = numAgents
        self.dt = dt
        self.continuous = continuous
        self.boundaryMap = boundaryMap
        if(self.enablePlotting):
            self.visualizer = PedsimVisualizer(plotdirections, plotaccelerations, plotRefreshRate, self.dt, enablePlotting, self.boundaryMap)
        
    # Advances the state to next iteration
    def simulate(self, state):
        start = time.perf_counter()
        for agent in state.agents:
            agent.update(state, self)
            # If user set dt via the -dt <deltatime> flag, use that. Otherwise use actual delta time as dt.                  
        if state.useFixedTimeStep:
            state.dt = state.fixedTimeStep
        else:
            state.dt = time.perf_counter() - start

        state.runningTimePerStep = time.perf_counter() - start
        state.time.append(state.time[-1] + state.dt)

    def run(self):

        #Generate data for use in each instance of pedsimstate
        NUM_STATES = 100
        variances = np.linspace(0.1, 1, NUM_STATES)
        means = np.linspace(0.5, 2.5, NUM_STATES)
        
        for state in [PedsimState(self.numAgents, self.dt, self.boundaryMap, means[stateIndex], variances[stateIndex]) for stateIndex in range(NUM_STATES)]:          
            # If plotting is enabled, run simulation until user presses quit
            if(self.enablePlotting):
                self.visualizer.clear()
                start = time.perf_counter()
                while not self.visualizer.terminate and state.numAgentsInGoal < self.numAgents:
                    if(self.visualizer.running):
                        self.simulate(state)
                    self.visualizer.visualize(state)
                print('Total time spent: %.2f' % (time.perf_counter() - start));
                    
            else:
                # If user passed --disableplotting no window will exist so no quit button
                start = time.perf_counter()
                while state.numAgentsInGoal < self.numAgents:
                    self.simulate(state)
                    self.saveData(state)
                self.saveDataToFile(state)
                print('Total time spent: %.2f' % (time.perf_counter() - start));
        
                
    def saveDataToFile(self, pedsimState):
        data = {"time":pedsimState.time, 
                "efficiencyLevels": pedsimState.efficiencyLevels}
        #TODO: Better name, currently used for testing only
        # pickle.dump(data, open ('nAgent%i_time%i_mean%.2f_var%.2f.p'%(pedsimState.numAgents, pedsimState.time[-1], pedsimState.mean,pedsimState.variance),"wb"))
    
    def saveData(self, state):
        # Saving the mean Efficiency of all agents in all timesteps
        tmpEfficiencyArray= []
        
        for agent in state.agents:
            efficiencyLevel = np.linalg.norm(agent.preferredVelocity) / np.linalg.norm(agent.velocity)
            efficiencyLevel = efficiencyLevel/self.numAgents
            tmpEfficiencyArray.append(efficiencyLevel)

        efficiencyLevel = np.sum(tmpEfficiencyArray)
        state.efficiencyLevels.append(efficiencyLevel)
        
        #TODO: Make a correct calculation of mean Discomfort -> DiscomfortLevel
        #tmpDiscomfortVector = []
        #for agent in state.agents:
        #    agentDiscomfortLevel = np.norm(agent.preferredVelocity)

        
def main():   
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", help="Sets the number of agents", type=int, default=100)
    parser.add_argument("-r", help="Sets the plot refresh rate", type=int, default=16)
    parser.add_argument("-dt", help="Sets delta time to fixed rate", type=float, default=0.0)
    parser.add_argument("--direction", help="Plot directions of agents", action='store_true')
    parser.add_argument("--acceleration", help="Plot accelerations of agents", action='store_true')
    parser.add_argument("--disableplotting", help="Disables plotting", action='store_true')
    parser.add_argument("--continuous", help="Resets x-coordinate after goal", action='store_true')
    parser.add_argument("-map", help="Sets map", type = int, default = 1 )
    #paser.add_argument("--savedata", help="Save data from simulation", type=bool, default = false)
    args = parser.parse_args()

    # Instansiate and run model
    bMap = Boundarymap()
    boundaryMap = bMap.boundaryMap1()
    
    # Instansiate and run model
    pedsim = Pedsim(args.n, args.direction, args.acceleration, args.r, args.dt, not args.disableplotting, args.continuous, boundaryMap)
    pedsim.run()

if __name__ == "__main__":
    main()