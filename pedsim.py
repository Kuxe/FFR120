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
    state = None
    visualizer = None

    # If enablePlotting=False, do not plot at all. Dont even create a window.
    # This is usefulf for running several simulations in parallel without wasting memory on GUI.
    enablePlotting = None

    
    #Variables to save data from..
    #TODO: Get better variablenames
    time = [0]
    relativeSpeedChanges = []
    
    
    
    def __init__(self, numAgents, plotdirections, plotaccelerations, plotRefreshRate, dt, enablePlotting, boundaryMap):
        self.enablePlotting = enablePlotting
        if(self.enablePlotting):
            self.visualizer = PedsimVisualizer(plotdirections, plotaccelerations, plotRefreshRate, dt, enablePlotting)
        self.state = PedsimState(numAgents, dt, boundaryMap)
        
    # Advances the state to next iteration
    def simulate(self, state):
        for agent in state.agents:
            start = time.perf_counter()
            agent.update(state)
            # If user set dt via the -dt <deltatime> flag, use that. Otherwise use actual delta time as dt.                  
            if state.useFixedTimeStep:
                state.dt = state.fixedTimeStep
            else:
                state.dt = time.perf_counter() - start
    
            #Only Works properly when Fixed Timesteps            
            self.time.append(self.time[-1] + state.dt)

    def run(self):
        # If plotting is enabled, run simulation until user presses quit
        if(self.enablePlotting):
            while not self.visualizer.terminate:
                if(self.visualizer.running):
                    self.simulate(self.state)
                self.visualizer.visualize(self.state)
                
                
        else:
            # If user passed --disableplotting no window will exist so no quit button
            # hence let simulation run for 10000 iterations
            # TODO: Replace with reasonable stop condition (ie all agents reached goal)
            agentsStillInProgress = 0
            while agentsStillInProgress < 100:
                self.simulate(self.state)
                self.saveData(self.state)
                agentsStillInProgress += 1
            self.saveDataToFile()
        
                
    def saveDataToFile(self):
        data = {"time":self.time, 
                "relativeSpeedChanges": self.relativeSpeedChanges}
        #TODO: Better name, currently used for testing only
        pickle.dump(data, open ("savedData.p","wb"))
    
    def saveData(self, state):
        tmpSpeedVector = []
        for agent in state.agents:
            agentRelativeSpeed = np.linalg.norm(agent.preferredVelocity) / np.linalg.norm(agent.velocity)
            tmpSpeedVector.append(agentRelativeSpeed)
        meanSpeed = np.mean(tmpSpeedVector)
        self.relativeSpeedChanges.append(meanSpeed)

        
def main():   
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", help="Sets the number of agents", type=int, default=100)
    parser.add_argument("-r", help="Sets the plot refresh rate", type=int, default=16)
    parser.add_argument("-dt", help="Sets delta time to fixed rate", type=float, default=0.0)
    parser.add_argument("--direction", help="Plot directions of agents", action='store_true')
    parser.add_argument("--acceleration", help="Plot accelerations of agents", action='store_true')
    parser.add_argument("--disableplotting", help="Disables plotting", action='store_true')
    parser.add_argument("--map", help="Sets map", type = int, default = 1 )
    #paser.add_argument("--savedata", help="Save data from simulation", type=bool, default = false)
    args = parser.parse_args()

    # Instansiate and run model
    bMap = Boundarymaps()
    boundaryMap = bMap.boundaryMap1()
    
    # Instansiate and run model
    pedsim = Pedsim(args.n, args.direction, args.acceleration, args.r, args.dt, not args.disableplotting, boundaryMap)
    pedsim.run()

if __name__ == "__main__":
    main()