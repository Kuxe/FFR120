# -*- coding: utf-8 -*-
import numpy as np

class Boundarymaps:
    def boundaryMap1(self):
        boundarymap1 = np.zeros((7,15), dtype = 'bool')
        boundarymap1[0,:] = 1 
        boundarymap1[-1,:] = 1
        boundarymap1[:,0] = 1
        boundarymap1[:,-1] = 1
        return boundarymap1
    
