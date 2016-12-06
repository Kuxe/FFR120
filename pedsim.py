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

# PROTIP: 
# python -m cProfile -s cumtime pedsim.py --disableplotting --continuous -n 2 -dt 0.07 > profile.txt

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
        state.nTimesteps +=1

    def run(self):

        #Generate data for use in each instance of pedsimstate

        NUM_MEANS = 20
        NUM_VARIANCES = 20
        AVG_NUM_GOALS_PER_AGENT = 3; #Each agent should on average enter goal 10 times, so 20 agents => 200 goals should be measured before terminating
        NUMBER_OF_MEANS = 20
        
        variances = np.linspace(0.1, 0.6, NUM_MEANS)
        means = np.linspace(0.8, 1.8, NUM_VARIANCES)
        efficiencies =  []

        allMeans = []
        allVariances = []
        lastTime = 0        
        counter = 0
        
        pedsimStates = []
        for i in range(len(means)):
            for j in range(len(variances)):
                pedsimStates.append(PedsimState(self.numAgents, self.dt, self.boundaryMap, means[i], variances[j]))
                                
        numRuns = 0;          
        for state in pedsimStates:          
            # If plotting is enabled, run simulation until user presses quit
            # TODO MASSIVE BUG MAY HAPPEN HERE
            tmpEfficiencies = []
            tmpDiscomforts = []
            for i in range(NUMBER_OF_MEANS):
                state.__init__(self.numAgents, self.dt, self.boundaryMap, state.mean, state.variance)
                if(self.enablePlotting):
                    self.visualizer.clear()
                    start = time.perf_counter()
                    while not self.visualizer.terminate and state.numAgentsInGoal < (self.numAgents if not self.continuous else self.numAgents* AVG_NUM_GOALS_PER_AGENT):
                        if(self.visualizer.running):
                            self.simulate(state)
                            self.saveRunData(state)
                        self.visualizer.visualize(state)            
                else:
                    # If user passed --disableplotting no window will exist so no quit button
                    start = time.perf_counter()
                    while state.numAgentsInGoal < (self.numAgents if not self.continuous else self.numAgents*AVG_NUM_GOALS_PER_AGENT):
                        self.simulate(state)
                        self.saveRunData(state)
                efficiency = self.saveData(state)
                print('%.2f percentage, Efficiency: %f' % (100.0*numRuns / (len(pedsimStates)*NUMBER_OF_MEANS), efficiency))
                numRuns += 1
                tmpEfficiencies.append(efficiency)
            print('Total time spent: %.2f' % (time.perf_counter() - start),'  Approx time left: %.1f' %((lastTime + (time.perf_counter() - start)*(len(pedsimStates)-counter))/2.0))
            lastTime = (time.perf_counter() - start)*len(pedsimStates)
            efficiencies.append(np.mean(tmpEfficiencies))
            allMeans.append(state.mean)
            allVariances.append(state.variance)
            counter += 1
            
        print(np.shape(means),np.shape(variances),np.shape(efficiencies))
        self.saveDataToFile(allMeans,allVariances,efficiencies)
        
    def saveRunData(self, state):
        for agent in state.agents:
            agent.cumSpeed += np.linalg.norm(agent.velocity)
            
    def saveData(self, state):
        efficiency = 0
        for agent in state.agents:
            efficiency += (agent.cumSpeed/state.nTimesteps) *(1.0/ np.linalg.norm(agent.preferredVelocity))
        return efficiency/self.numAgents
        
    def saveDataToFile(self,means, variances, efficiencies):
        np.savetxt('text.txt',np.c_[means,variances,efficiencies])
    
    def saveEfficiency(self, state):
        tmpEfficiencyArray= []
        
        for agent in state.agents:
            velocitySum = 0;
            for i in range(len(agent.velocityInTimeX)):
                tmpVelocity = np.array([agent.velocityInTimeX[i], agent.velocityInTimeY[i]])
                preferredDirection = agent.preferredVelocity/np.linalg.norm(agent.preferredVelocity)
                velocitySum += np.dot(tmpVelocity,preferredDirection)
            averageVelocity = velocitySum/len(agent.velocityInTimeX)
            efficiencyLevel = averageVelocity /np.linalg.norm(agent.preferredVelocity)
            efficiencyLevel = efficiencyLevel/self.numAgents
            tmpEfficiencyArray.append(efficiencyLevel)
    
        efficiencyLevel = np.sum(tmpEfficiencyArray)
        state.efficiencyLevels.append(efficiencyLevel)
        
    def saveDiscomfort(self, state):
        
        #TODO: Make a correct calculation of mean Discomfort -> DiscomfortLevel
        tmpDiscomfortVector = []
        for agent in state.agents:
            velocitySum = 0;
            velocitySquared = 0;
            for i in range(len(agent.velocityInTimeX)):
                tmpVelocity = np.linalg.norm(np.array([agent.velocityInTimeX[i], agent.velocityInTimeY[i]]))
                velocitySum += tmpVelocity
                velocitySquared += tmpVelocity**2
            velocitySum = velocitySum/len(agent.velocityInTimeX)
            velocitySquared = velocitySquared/len(agent.velocityInTimeX)
            agentDiscomfortLevel = 1 - (velocitySum**2)/velocitySquared;
            agentDiscomfortLevel = agentDiscomfortLevel/self.numAgents
            tmpDiscomfortVector.append(agentDiscomfortLevel)
        
        discomfortLevel = np.sum(tmpDiscomfortVector)
        state.discomfortLevels.append(discomfortLevel)
        
    def saveData(self, state):
        
        for agent in state.agents:
            agentVelocityX = agent.velocity[0]
            agentVelocityY = agent.velocity[1]
            agent.velocityInTimeX.append(agentVelocityX)
            agent.velocityInTimeY.append(agentVelocityY)
        
        

        
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