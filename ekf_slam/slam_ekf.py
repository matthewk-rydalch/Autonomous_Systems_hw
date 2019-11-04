#initialization
    #robot starts in its own reference fram all landmarks unknown

##compare with ekf
#prediction steps
    #g
        #we have the pose states but we also have the landmark states
        #the landmarks should not be affected in the prediciton steps
        #use an identity matrix of the appropriate size with zeros appended on it to make there be no change to the markers' states.  See Slides for update the state space.
    #covariance:
        #jacobian of the motion (pose jacobian) is the same, but an identity matrix is used for the markers.  See slides update covariance
        #previous sigma is a matrix of covariances of the pose, markers, and their correlations.
    #extra steps
        #Fx needs to be applied as in slide 41.  Rtx is the pose noise.  We don't use the measurment noise like we used before.  They won't be functions of the speed.

#measurment update
    #when you first see a landmark you have to initialize it
    #we use the belief of the robot location to intialize the landmarks
    #use low-dim space to get the jacobian of markers slide 46
    #we do little jacobian calculations and then we map them into the complete jacobian.
    #you don't necissarily need to make these f matrices and multiply them.  You can just place them where you know they should go.
    #Once we have H everything mostly works the same.
