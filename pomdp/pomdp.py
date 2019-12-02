import numpy as np

class POM:
    def __init__():

    def value_function(T):
        #G is a struct of lines/vector (alpha vectors) v and a control
        # in example the points of the line segment are the probabillities of being in each state.
        #k is the index of v, kth linear function.
        #change G's to be Y's
        G = np.zeros((1,n)) #how big? This is wrong
        #his algorithm did fine with computational speed and these loops
        for tau in range(1,T): #check this range when you understand the horizon.  Should it be 1 or 0
            Gp = zeros(()) #how big?
            for u in ut: #all control actions u #look at effect of control
                for z in zt: #all measurments z #look at effect of measurment
                    for j in range(1,N): #we are seeing what happens to each line with each control and measurment
                        v[u][z][j] = np.sum(v[i]*p(z|xi)*p(xi|u,xj) #what are the k superscripts on the v's.  What are the other elements values of v[i] (3d)
                        #The probabilities just come from being in state x and getting a certain z and such ...
            for u in ut:
                for k = np.ones(len(k)):np.ones(len(k))*abs(G): #need to modify the syntax.  M is the number of measurments
                #this is essentially two for loops nested from 1 to the number of lines that we have for each measurement (in our case two)
                #k(1) means k for the first measurement
                    for i in range(1,N):
                        vp[i] = g*(r(xi,u)+sum(v[u][z][i])) #This is the same equation from mdp
                    Gp[] = [u,vp] #adding (u:v'1, v'2, ...) to Gp
            #optional prune Gp.  These problems get really complicated so you should prune
            #in his he doesn't prune on first time horizon, but he prunes for all the others.
            G = Gp
        return G

    def policy():
        uhat = argmax(sum(v*p)) #need to correct this.

    def prune():
        #divide lines up into negative and positive slope lines.  Start with imediate reward function and look for each intersection.
        #do the same for the positive slope lines
        #any lines below the intersection of the positive and negative slopes you can remove the shallowest lines.
        #see notes
