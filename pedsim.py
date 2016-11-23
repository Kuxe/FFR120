import numpy as np
import time
import argparse
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import warnings
warnings.filterwarnings("ignore", category=np.VisibleDeprecationWarning) 

# Each agent has a position, velocity and preferred velocity
# The Pedestrian simulator Pedsim initializes several agents.
# Updating Pedsim will in turn update each agent.
# Parameters to the agent update method informs the agent of the
# simulation state such that the agent can make decisions during that
# particular update. 

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
        # TODO: Implement according to 
        return np.array([0, 0])
    
    def repulsiveInteractions(self, agents):
        # Almost Coulomb potential, Q = 1 temporary?
        sum = np.array([0.0, 0.0]);
        for agent in agents:
            if(agent != self):
                rab = self.position - agent.position
                sum += rab / np.dot(rab, rab)
        return sum
    
    def attractionEffects(self, attractors):
        # TODO: Implement according to paper
        return np.array([0, 0])
        
    def update(self, agents, boundaries, attractors, dt):
        tmpPos = np.copy(self.position);
        self.acceleration = self.behavioral(agents, boundaries, attractors)
        self.velocity += self.acceleration
        self.position += self.velocity * dt
        return np.linalg.norm(self.position-tmpPos)

class Pedsim:

    # Declare list of agents
    agents = []
    terminate = False
    running = True
    boundaries = None
    attractors = None
    
    app = None
    w = None
    runBtn = None
    quitBtn = None
    togglePlottingBtn = None
    agentPlot = None
    dataPlot = None
    dataCurve = None
    layout = None
    plotdirections = None
    plotaccelerations = None
    plotRefreshRate = None
    useFixedTimeStep = False
    fixedTimeStep = None
    enablePlotting = None
    
    totalDistanceTravelled = 0
    
    def __init__(self, numAgents, plotdirections, plotaccelerations, plotRefreshRate, dt, enablePlotting):
        if(dt != 0.0):
            self.useFixedTimeStep = True
            self.fixedTimeStep = dt

        ## Always start by initializing Qt (only once per application)
        self.app = QtGui.QApplication([])
        
        ## Define a top-level widget to hold everything
        self.w = QtGui.QWidget()
        
        ## Create some widgets to be placed inside
        self.runBtn = QtGui.QPushButton('Pause')
        self.quitBtn = QtGui.QPushButton('Quit')
        self.togglePlottingBtn = QtGui.QPushButton('Disable plotting' if enablePlotting else 'Enable plotting')
        self.quitBtn.clicked.connect(self.onQuit)
        self.runBtn.clicked.connect(self.toggleRunning)
        self.togglePlottingBtn.clicked.connect(self.togglePlotting)
        
        # Create a PlotWidget which shows position of agents as circles via scatterplotting
        self.agentPlot = pg.PlotWidget()
        self.agentPlot.showGrid(True, True, 0.3)
        self.agentPlot.setXRange(-1, 1)
        self.agentPlot.setYRange(-1, 1)
        self.agentPlot.setDownsampling(mode='peak')
        
        self.dataPlot = pg.PlotWidget()
        self.dataPlot.showGrid(True, True, 0.3)
        self.dataPlot.setDownsampling(mode='peak')
        self.dataPlot.setClipToView(True)
        self.dataCurve = self.dataPlot.plot()
        
        
        ## Create a grid layout to manage the widgets size and position
        self.layout = QtGui.QGridLayout()
        self.w.setLayout(self.layout)
        
        ## Add widgets to the layout in their proper positions
        self.layout.addWidget(self.runBtn, 0, 0)
        self.layout.addWidget(self.togglePlottingBtn, 1, 0)
        self.layout.addWidget(self.quitBtn, 2, 0)
        self.layout.addWidget(self.agentPlot, 0, 1, 1, 1)  # plot goes on right side, spanning 3 rows
        self.layout.addWidget(self.dataPlot, 0, 2, 1, 1)

        self.w.show()
        
        self.plotdirections = plotdirections
        self.plotaccelerations = plotaccelerations
        self.plotRefreshRate = plotRefreshRate
        self.enablePlotting = enablePlotting
        
        # TODO: Give agents some reasonable starting values (currently just randomize every member in range [-1, 1])
        self.agents = [Agent(np.random.random(2)*2-1, (np.random.random(2)*2-1), (np.random.random(2)*2-1)) for i in range(numAgents)]

            
    def onQuit(self):
        self.terminate = True
        
    def toggleRunning(self):
        self.running = not self.running
        self.runBtn.setText("Pause " if self.running else "Run")

    def togglePlotting(self):
        self.enablePlotting = not self.enablePlotting
        self.togglePlottingBtn.setText("Disable plotting " if self.enablePlotting else "Enable plotting")
            
    def run(self):
        
        data3 = np.empty(100)
        ptr3 = 0
        
        plotTime = time.perf_counter() - time.perf_counter();
        dt = time.perf_counter() - time.perf_counter();
        
        while not self.terminate:
            start = time.perf_counter()
            self.app.processEvents() # Such as moving the window, pressing buttons etc
            if(self.running == True):
                
                # Update all agents
                for agent in self.agents:
                    
                    #Dummyvariable could really be a tuple or anything that we want to plot
                    dummyvariable = agent.update(self.agents, self.boundaries, self.attractors, dt)
                    self.totalDistanceTravelled += dummyvariable
                    
                # Update plot, 60fps
                if(self.enablePlotting and (time.perf_counter() - plotTime)*1000 > self.plotRefreshRate):
                    xs = [agent.position[0] for agent in self.agents]
                    ys = [agent.position[1] for agent in self.agents]
                    self.agentPlot.plot(xs, ys, pen=None, symbol='o', clear=True)

                    # FIXME: !!!MAJOR CHOKEPOINT!!! totally worthless
                    if(self.plotdirections):
                        for agent in self.agents:
                            self.agentPlot.plot([agent.position[0], agent.position[0] + agent.velocity[0]/20], [agent.position[1], agent.position[1] + agent.velocity[1]/20], pen={'color': (100,241,64), 'width': 0.5})
                    if(self.plotaccelerations):
                        for agent in self.agents:
                            self.agentPlot.plot([agent.position[0], agent.position[0] + agent.acceleration[0]/20], [agent.position[1], agent.position[1] + agent.acceleration[1]/20], pen={'color': (241, 100, 64), 'width': 0.5})
                    
                    data3[ptr3] = self.totalDistanceTravelled
                    ptr3 += 1
                    if ptr3 >= data3.shape[0]:
                        tmp = data3
                        data3 = np.empty(data3.shape[0] * 2)
                        data3[:tmp.shape[0]] = tmp      
                    
                    self.dataCurve.setData(data3[:ptr3])
                            
                    plotTime = time.perf_counter()
                
            # If user set dt via the -dt <deltatime> flag, use that. Otherwise use actual delta time as dt.                  
            dt = self.fixedTimeStep if self.useFixedTimeStep else time.perf_counter() - start

            self.agentPlot.setTitle("%.2fms" % round(dt*1000, 2))
            self.dataPlot.setTitle("%.2fm" % self.totalDistanceTravelled)
            
        
def main():   
    parser = argparse.ArgumentParser()
    parser.add_argument("-n", help="Sets the number of agents", type=int, default=100)
    parser.add_argument("-r", help="Sets the plot refresh rate", type=int, default=16)
    parser.add_argument("-dt", help="Sets delta time to fixed rate", type=float, default=0.0)
    parser.add_argument("--direction", help="Plot directions of agents", action='store_true')
    parser.add_argument("--acceleration", help="Plot accelerations of agents", action='store_true')
    parser.add_argument("--disableplotting", help="Disables plotting", action='store_true')
    args = parser.parse_args()
    
    # Instansiate and run model
    pedsim = Pedsim(args.n, args.direction, args.acceleration, args.r, args.dt, not args.disableplotting)
    pedsim.run()

if __name__ == "__main__":
    main()