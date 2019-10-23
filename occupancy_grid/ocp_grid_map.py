from IPython.core.debugger import set_trace
from importlib import reload
import numpy as np

class ocp_grid_map:
    #l0 is the belief at the beginning (.5).  Log of it?
    def ocp_grid_map(self, lprev_i, xt, zt):
        #lprev_i is the log odds ratio at the previous time
        #l = log(p/(1-p))
        for m in cells
            if m in percep_field(zt):
                lt_i = lprev_i+self.inverse_sensor_model(m,xt,zt)-l0
            else:
                lt_i = lprev_i

        return(lt_i)

    def inverse_sensor_model(self, mi, xt, zt):
        #mi is the cell
        #xi, yi are center of mi
        #th is beam width?
        r = np.sqrt((xi-x)**2+(yi-y)**2) #range
        phi = atan2(yi-y,xi-x) - th #bearing
        k = argmin(abs(phi-th_j_sens)) #finding out which beam would reach that cell
        #beta is beam width, thk_sns is the kth sensor
        #alpha is the thickness of anticipated obstacle or grid cell dimension? value is given
        if r>min(zmax,ztk+alpha/2) or abs(phi-thk_sns)>beta/2: #check if cell is in view
            return l0 #not detecting cell

        if ztk <zmax and abs(r-ztk)<alpha/2: #check if cell is hit
            return l_occ

        if r <= ztk: #check if cell is free
            return l_free

        #in case none of these l's conditions are met return 0?  I think that is what he does.
