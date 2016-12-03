import numpy as np
import time
import argparse
import pyqtgraph as pg
from pyqtgraph.Qt import QtGui, QtCore
import warnings
warnings.filterwarnings("ignore", category=np.VisibleDeprecationWarning)

# Class which visualizes a PedsimState using the python library pyqtgraph
class PedsimVisualizer:
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
    enablePlotting = None
    data3 = None
    ptr = None
    running = True

    terminate = False
    plotTime = time.perf_counter() - time.perf_counter();

    def __init__(self, plotdirections, plotaccelerations, plotRefreshRate, dt, enablePlotting, boundaryMap):
        self.data3 = np.empty(100)
        self.ptr3 = 0
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
        boundaryMapMaxLen = np.max(np.shape(boundaryMap));
        self.agentPlot.setXRange(0, boundaryMapMaxLen)
        self.agentPlot.setYRange(-boundaryMapMaxLen/2, boundaryMapMaxLen/2)
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
        #self.layout.addWidget(self.dataPlot, 0, 2, 1, 1)

        self.w.show()
            
        self.plotdirections = plotdirections
        self.plotaccelerations = plotaccelerations
        self.plotRefreshRate = plotRefreshRate
        self.enablePlotting = enablePlotting
        
    def onQuit(self):
        self.terminate = True
        
    def toggleRunning(self):
        self.running = not self.running
        self.runBtn.setText("Pause " if self.running else "Run")

    def togglePlotting(self):
        self.enablePlotting = not self.enablePlotting
        self.togglePlottingBtn.setText("Disable plotting " if self.enablePlotting else "Enable plotting")

    def visualize(self, state):

        self.app.processEvents() # Such as moving the window, pressing buttons etc
        # Update plot, 60fps
        if(self.running and self.enablePlotting and (time.perf_counter() - self.plotTime)*1000 > self.plotRefreshRate):
            
            x = [i for j in range(len(state.boundaryMap[:,0])) \
                          for i in range(len(state.boundaryMap[0,:])) if state.boundaryMap[len(state.boundaryMap[:,0])-1-j,i] == True]
            y = [j for j in range(len(state.boundaryMap[:,0])) \
                          for i in range(len(state.boundaryMap[0,:])) if state.boundaryMap[len(state.boundaryMap[:,0])-1-j,i] == True]                              
            self.agentPlot.plot(x, y, pen=None, symbol='s', clear=True)

            xs = [agent.position[0] for agent in state.agents]
            ys = [agent.position[1] for agent in state.agents]
            self.agentPlot.plot(xs, ys, pen=None, symbol='o', clear=False)

            #Dummyvariable could really be a tuple or anything that we want to plot
            self.data3[self.ptr3] = state.totalDistanceTravelled
            self.ptr3 += 1
            if self.ptr3 >= self.data3.shape[0]:
                tmp = self.data3
                self.data3 = np.empty(self.data3.shape[0] * 2)
                self.data3[:tmp.shape[0]] = tmp      
                    
            self.dataCurve.setData(self.data3[:self.ptr3])
                            
            self.plotTime = time.perf_counter()

            # FIXME: !!!MAJOR CHOKEPOINT!!! totally worthless
            if(self.plotdirections):
                for agent in state.agents:
                    self.agentPlot.plot([agent.position[0], agent.position[0] + agent.velocity[0]/20], [agent.position[1], agent.position[1] + agent.velocity[1]/20], pen={'color': (100,241,64), 'width': 0.5})
            if(self.plotaccelerations):
                for agent in state.agents:
                    self.agentPlot.plot([agent.position[0], agent.position[0] + agent.acceleration[0]/20], [agent.position[1], agent.position[1] + agent.acceleration[1]/20], pen={'color': (241, 100, 64), 'width': 0.5})

            self.agentPlot.setTitle("Running time per iteration: %.2fms, num agents in goal: %i" % (round(state.runningTimePerStep*1000, 2), state.numAgentsInGoal))
            self.dataPlot.setTitle("%.2fm" % state.totalDistanceTravelled)
