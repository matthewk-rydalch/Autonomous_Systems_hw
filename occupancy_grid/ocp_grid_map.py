from IPython.core.debugger import set_trace
from importlib import reload
import numpy as np
import math
from importlib import reload, import_module
utils = reload(import_module("utils"))

class OcpGridMap:
    def __init__(self):
        params = utils.read_param('map_params.yaml')
        for key, value in params.items():
            setattr(self,key,value)
        p0 = self.map0
        p_occ = self.hit
        p_free = self.nohit
        self.l0 = np.log(p0/(1-p0))
        self.l_occ = np.log(p_occ/(1-p_occ))
        self.l_free = np.log(p_free/(1-p_free))
        self.free = 0
        self.cell0 = 0
        self.occ = 0
        self.beta = self.beta*math.pi/180
        #

    def ocp_grid_map(self, lprev_i, xt, zt):
       
        cells = lprev_i.shape
        i = 0
        lt_i = lprev_i
        for m in range(cells[0]):
            for n in range(cells[1]):
                cell = [m, n]
                lt_i[m][n] = lprev_i[m][n]+self.inverse_sensor_model(cell,xt,zt)-self.l0
        # print('free = ', self.free)
        # print('occ = ', self.occ)
        # print('cell0 = ', self.cell0)
        return(lt_i)

    def inverse_sensor_model(self, mi, xt, zt):
        ri = np.sqrt((mi[0]-xt[0])**2+(mi[1]-xt[1])**2) #range
        #arctan2?
        phi = np.arctan((mi[1]-xt[1])/(mi[0]-xt[0])) - xt[2] #bearing
        k = np.argmin(abs(phi-zt[1])) #finding out which beam would reach that cell

        if ri > np.minimum(self.z_max,zt[0][k]+self.alpha/2) or abs(phi-zt[1][k])>self.beta/2: #check if cell is in view 
            # print('l0')
            # self.cell0 = self.cell0+1
            return self.l0 #not detecting cell
        elif zt[0][k] < self.z_max and abs(ri-zt[0][k])<self.alpha/2: #check if cell is hit
            # print('l_occ')
            # self.occ = self.occ+1
            return self.l_occ
        elif ri <= zt[0][k]: #check if cell is free
            # print('l_free')
            # self.free = self.free+1
            return self.l_free

        #in case none of these l's conditions are met return 0?  I think that is what he does.
        return(0)
