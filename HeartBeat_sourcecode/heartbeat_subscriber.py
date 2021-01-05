#!/usr/bin/env python3 

import rospy
from std_msgs.msg import String
from std_msgs.msg import Time
from std_msgs.msg import Duration

# Global Variable to store the reception time stamp of the previous packet
time_stamp_previous_packet = Time()
# Initializing its Value to Zero
time_stamp_previous_packet.data.secs= 0
time_stamp_previous_packet.data.nsecs=0

#Packet counts 

tot_packets_received = 0
tot_packets_exceeded_delay = 0



def callback_heart_beat(msg):

    global time_stamp_previous_packet

    global tot_packets_received
    global tot_packets_exceeded_delay

    tot_packets_received += 1 
    
    sent_time=msg   # data from the sender will be saved at this variable. type : Time
    # received_time captures the time stamp at which the packet packet is received 
    received_time = Time() 
    received_time.data= rospy.Time.now() # captures the current time stamp i.e) The time at which the packet reaches the Sub node 
   
   
#Calculating the time difference between two consective packets :
    # Setting the reference Duration as 1 sec     
    # reference_time_stamp is of class "duration" and its value is 1 second 
    reference_time_stamp = Duration()
    reference_time_stamp.data.secs=1
    reference_time_stamp.data.nsecs=0

 
    # Time Difference between two consecutive Packet
    #feedback_ststus stores the time delay between recption of 2 consective packets 
    feedback_status = Duration()
    feedback_status.data=received_time.data - time_stamp_previous_packet.data
    print("\n")
    print("The time difference between two conseuctive packets is " + str(feedback_status.data)+ " ns")
    
    # Condition to check whether the Delay between two consecutive packets excceds 1 sec 
    
    if (feedback_status.data >= reference_time_stamp.data) :
        tot_packets_exceeded_delay += 1
        print("------------------------------------------------------------------------")
        print("*************The Network is experiencing the Undesirable Delay**********")
        print("-------------------------------------------------------------------------")
        pub_hb_reply.publish(feedback_status) # Publishes when network delay is exceeded more than 1 sec 
   
   
   # Printing the details about Sending and Receiving time od the packet
   # uncomment the following block for more information  
    '''print("\n\n\n")
    print("\t\t\t\t-----Start of the Packet-----")
    print("The packet sent from pub node  at ")
    print(sent_time)
    print("The packet received at sub node at ")
    print(received_time)'''
   # print(received_time.data) # prints in string format 

    # Calculating the Latency from sender to receiver 

    latency_current_packet = Duration()
    latency_current_packet.data = received_time.data - sent_time.data
    #print("The Differenc is: ")
    #print(latency_current_packet)
    print("The end-to end latency for this packet  is " + str(latency_current_packet.data) + " ns.")
    #print("\t\t\t\t ------End of the Packet-----")

# Changing the Value of the previous time stamp :
    
    #Debugging Block
    '''
    print("Before")
    print(time_stamp_previous_packet)
    #time_stamp_previous_packet.data=received_time.data
    print("After")
    print(time_stamp_previous_packet)'''

    # packet statistics 
    print("-----Statistics-----")
    print("total packets received : total packets exceeded delay  "+ str(tot_packets_received)+ " : "+ str(tot_packets_exceeded_delay))
    print("\n\n")
    print("\t\t\t\t------")

    # storin the value for next iteration 
    time_stamp_previous_packet.data=received_time.data





if __name__ == "__main__":
    rospy.init_node("heart_beat_subscriber",anonymous=True)
    sub = rospy.Subscriber("/heart_beat", Time, callback_heart_beat)
    pub_hb_reply= rospy.Publisher("/heart_beat_feedback", Duration, queue_size=10)
    rospy.spin()
