#!/usr/bin/env python
#Move from a start point to a specified goal point. The goal point is hardcoded(normally we would get this from the 
#road segmentation algorithm). If an obstacle is in the FOV of the camera we stop. Stopping is achieved by
# publishing the current position of the robot as the goal point. As the object moves out of the FOV of 
# the camera, the robot continues until it reaches the destination point.
import rospy
import actionlib
# Brings in the .action file and messages used by the move base action
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

class navStack():
    def __init__(self):
        self.x = None
        self.y = None
        self.client = None #MoveBase Client
        self.flag = False
        self.obj_flag_ls = [0,0,0,0,0]
        self.count = 0 #Counter to keep track of the goalpoints in the list self.goal_ls
        self.global_count = 0 #Counter to stop after 'n' rounds
        rospy.init_node('movebase_client_py')
        #GoalPoint list would help us in generalizing to any number of waypoints
        self.goal_ls = [self.define_goalpoint(x=5.0, y=0.0, z=0.0, w=1.0)]
        #Subscribe to the /amcl_pose topic
        rospy.Subscriber("amcl_pose",PoseWithCovarianceStamped,self.pose_callback)
        #Subscribe to the /obj_det topic
        rospy.Subscriber("obj_det",String,self.obj_callback)

        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        # # Waits until the action server has started up and started listening for goals.
        wait = self.client.wait_for_server()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Sending goals...")
        self.movebase_client()

    #Define Callbacks
    def pose_callback(self, data):
        print("Inside pose_callback")
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
    
    def obj_callback(self, data):
    
        """ Callback for the obj_det topic.
        The topic publishes True when a person is in its FOV else it publishes False.
        """

        #Compute an average of the number of True/False detections 
        #If avg>0.5, stop the robot else continue towards the goal point
        if(data.data == "True"):
            self.obj_flag_ls.append(1)
        elif(data.data == "False"):
            self.obj_flag_ls.append(0)

        if(len(self.obj_flag_ls) > 5):
            self.obj_flag_ls.pop(0)
        
        if(float(sum(self.obj_flag_ls))/float(len(self.obj_flag_ls)) > 0.5):
            # self.flag = True
            #Stop the robot at its current position
            goal = self.define_goalpoint(x=self.x,y=self.y,z=0.0,w=1.0)
            self.client.send_goal(goal,self.done_cb)

        

        
        #Debug Data
        # print("sum(ls):",sum(self.obj_flag_ls))
        # print("len(ls):",len(self.obj_flag_ls))
        # print("Division result:",float(sum(self.obj_flag_ls))/float(len(self.obj_flag_ls)))
        # print(self.obj_flag_ls)
            
            
    
    def done_cb(self, status, result):
        if status == 2:
            # rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request after it started executing, completed execution!")
            rospy.loginfo("Received a cancel request after execution")

        if status == 3:
            # rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached") 
            rospy.loginfo("Goal pose reached")
            #Print statements for debug
            # print("self.x",self.x)
            # print("Goal Position:",self.goal_ls[self.count].target_pose.pose.position.x - 0.1)

            if(self.x < (self.goal_ls[self.count].target_pose.pose.position.x - 0.5)): #TODO: Extend this to the y axis also
                rospy.loginfo("Sending the goal pose")
                self.client.send_goal(self.goal_ls[self.count],self.done_cb)
            else:
                rospy.signal_shutdown("Final goal pose reached!")
                return


        if status == 4:
            # rospy.loginfo("Goal pose "+str(self.goal_cnt)+" was aborted by the Action Server")
            # rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" aborted, shutting down!")
            rospy.loginfo("Goal pose aborted by Action Server")
            rospy.signal_shutdown("Goal pose aborted, shutting down!")          
            return

        if status == 5:
            # rospy.loginfo("Goal pose "+str(self.goal_cnt)+" has been rejected by the Action Server")
            # rospy.signal_shutdown("Goal pose "+str(self.goal_cnt)+" rejected, shutting down!")
            rospy.loginfo("Goal pose has been rejected by the Action Server")
            rospy.signal_shutdown("Goal pose rejected, shutting down!")
            return

        if status == 8:
            #rospy.loginfo("Goal pose "+str(self.goal_cnt)+" received a cancel request before it started executing, successfully cancelled!")
            rospy.loginfo("Goal pose received a cancel request before it started executing, successfully cancelled!")


    def define_goalpoint(self,x,y,z,w):
        # Creates a new goal with the MoveBaseGoal constructor
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        # Move 0.5 meters forward along the x axis of the "map" coordinate frame 
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = 0.0
        # No rotation of the mobile base frame w.r.t. map frame
        goal.target_pose.pose.orientation.w = 1.0
        return goal 


    def movebase_client(self):
        self.client.send_goal(self.goal_ls[self.count],self.done_cb)
        print("Client send the goal")
        rospy.spin()  

def main():
    navStack()

if __name__ == '__main__':
    main()
    