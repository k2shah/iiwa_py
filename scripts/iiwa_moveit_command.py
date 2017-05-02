#!/usr/bin/env python
import sys
import copy
import rospy
import std_msgs.msg
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose

class Commander:
    def __init__(self):
        #init nod
        rospy.loginfo("running moveit interface setup")
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('commander',anonymous=True)
        #shutdown handlers
        rospy.on_shutdown(self.release)
        #init interfaces
        self.robot = moveit_commander.RobotCommander()
        #while not rospy.is_shutdown():
        try:
            self.group = moveit_commander.MoveGroupCommander("manipulator", '/iiwa/robot_description' )
            #self.group = moveit_commander.MoveGroupCommander(rospy.get_param("move_group"))
        except e:
            print(e)
            rospy.logwarn("robot not connected")
        self.group.set_planning_time(0.5)
        
        #self.group.set_planning_time(rospy.get_param("plan_time"))
        self.group.set_planner_id("RRTConnectkConfigDefault")
        #get end effector
        try:
            self.endEff=rospy.get_param("~ee_link")
        except e:
            rospy.logwarn("No End effector passed")
            self.endEff="ee_link"

        #print dir(self.group) because moveit doc sucks

        self.group.set_end_effector_link(self.endEff)        
        #set homePose
        self.homePose=Pose()
        #pos
        self.homePose.position.x=0
        self.homePose.position.y=0
        self.homePose.position.z=1.306
        #ori
        self.homePose.orientation.x=0
        self.homePose.orientation.y=0
        self.homePose.orientation.z=0
        self.homePose.orientation.w=1

        #rospy.loginfo("Waiting for RVIZ")
        #rospy.sleep(10)
        #init pubs
        self.startState_pub = rospy.Publisher('/rviz/moveit/update_start_state', 
                                    std_msgs.msg.Empty, 
                                    queue_size=20) 
        self.dispTraj_pub = rospy.Publisher('/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory,
                                    queue_size=20)
        #init subs
        self.goalPose_sub=rospy.Subscriber('/move_group/goal_external', 
                                    Pose, self.goalPose_callback)
        self.reset(1); #go home


    def run(self):
        rospy.spin()
    
    def goalPose_callback(self, msg):
        rospy.loginfo("target pose acquired")
        targetPose=msg.data
        #check if pose
        if type(targetPose)!=Pose:
            rospy.logwarn("invalid pose requst ")
            return 
        #check if valid
        if self.planTraj(targetPose):
            rospy.loginfo("Valid Trajectory Found")
        else:
            rospy.loginfo("Navigation Failure: No trajectory to target pose")


        
    def testTraj(self):
        rospy.loginfo("running test")
        #init pose target
        dirc=1
        targetPose=Pose()
        targetPose=copy.copy(self.homePose)
        for i in range(2):
            #set pose
            #pos
            targetPose.position.x=.2*dirc
            targetPose.position.y=.2*dirc
            targetPose.position.z=.9
            print "target pose is:"
            print targetPose
            if self.planTraj(targetPose):
                self.group.go()
                rospy.sleep(3)
            dirc*=-1
            self.reset()

    def reset(self, val=0):
        #resets pose targets and planner
        self.group.clear_pose_targets()
        self.group.set_start_state_to_current_state()
        if val==1: #go home
            if self.planTraj(self.homePose):
                pass
                self.group.go()
                rospy.sleep(3)
                self.startState_pub.publish(std_msgs.msg.Empty())

    def getEndEff(self, verbose=True):
        cPose_ST=self.group.get_current_pose(self.endEff) #this returns a poseStamped msg
        cPose=cPose_ST.pose #unpack pose
        if verbose:
            rospy.loginfo("Current position is %2.3f, %2.3f %2.3f", 
                                            cPose.position.x, cPose.position.y, cPose.position.z) 
            rospy.loginfo("Current orientation is %2.3f, %2.3f %2.3f, %2.3f", 
                                            cPose.orientation.x,  cPose.orientation.y, cPose.orientation.z, cPose.orientation.w)
        return cPose 

    def planTraj(self, pose):
        self.reset()
        self.group.set_pose_target(pose)
        rospy.sleep(1)
        #rviz update start state
        self.startState_pub.publish(std_msgs.msg.Empty())
        #plan 
        plan=self.group.plan()
        if len(plan.joint_trajectory.points)==0:
            rospy.logwarn("Navigation Failure")
            return False
        else:
            dispTraj = moveit_msgs.msg.DisplayTrajectory()
            dispTraj.trajectory_start = self.robot.get_current_state()
            dispTraj.trajectory.append(plan)
            print(type(dispTraj.trajectory[-1]))
            print(dir(dispTraj.trajectory[-1]))
            #self.dispTraj_pub.publish(dispTraj);
            return True

    def release(self):
        #release planner
        rospy.logwarn("Commander Shutdown")
        moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    cmdr = Commander()
    #cmdr.testTraj()
    cmdr.run()