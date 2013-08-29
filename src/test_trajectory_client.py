import roslib; roslib.load_manifest('staubliTX60')
import rospy
import actionlib
import staubliTX60.msg


from copy import *

def set_trajectory_client(client = None):
    if client == None:
        #setup client
        client = actionlib.SimpleActionClient('setJointTrajectory', staubliTX60.msg.SetJointTrajectoryAction)
        client.wait_for_server()
    jointGoalList = list()      
    #set up joint trajectory params
    p = staubliTX60.msg.StaubliMovementParams(jointVelocity = 1.0,
    jointAcc = 1.0,
    jointDec= 1.0,
    endEffectorMaxTranslationVel =  9999.0,
    endEffectorMaxRotationalVel =  9999.0,
    movementType = 1)
    #Set up Joint Goals
    jointGoalList.append(staubliTX60.msg.JointTrajectoryPoint(jointValues = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], params = p))
    for i in range(30):
        jointGoalList.append(deepcopy(jointGoalList[-1]))
        jointGoalList[-1].jointValues[5] += .05
        
    actionGoal = staubliTX60.msg.SetJointTrajectoryGoal(jointTrajectory = jointGoalList)
    client.send_goal(actionGoal)
    
    return client, jointGoalList
    

    
