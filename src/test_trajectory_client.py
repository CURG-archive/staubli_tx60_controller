import roslib; roslib.load_manifest('staubli_tx60')
import rospy
import actionlib
import staubli_tx60.msg


from copy import *

def set_trajectory_client(client = None):
    if client == None:
        #setup client
        client = actionlib.SimpleActionClient('setJointTrajectory', staubli_tx60.msg.SetJointTrajectoryAction)
        client.wait_for_server()
    jointGoalList = list()      
    #set up joint trajectory params
    p = staubli_tx60.msg.StaubliMovementParams(jointVelocity = 1.0,
    jointAcc = 1.0,
    jointDec= 1.0,
    endEffectorMaxTranslationVel =  9999.0,
    endEffectorMaxRotationalVel =  9999.0,
    movementType = 1)
    #Set up Joint Goals
    jointGoalList.append(staubli_tx60.msg.JointTrajectoryPoint(jointValues = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], params = p))
    for i in range(30):
        jointGoalList.append(deepcopy(jointGoalList[-1]))
        jointGoalList[-1].jointValues[5] += .05
        
    actionGoal = staubli_tx60.msg.SetJointTrajectoryGoal(jointTrajectory = jointGoalList)
    client.send_goal(actionGoal)
    
    return client, jointGoalList
    

    
