# -*- coding: utf-8 -*-
import numpy as np

class Boundarymaps:
    def boundaryMap1(self):
        boundarymap1 = np.zeros((10,10), dtype = 'bool')
        boundarymap1[0,:] = 1 
        boundarymap1[9,:] = 1
        boundarymap1[:,0] = 1
        boundarymap1[:,9] = 1
        return boundarymap1

if __name__ == "__main__":
    bMap = Boundarymaps()
    map = bMap.boundaryMap1()
    print(map)
    
