#!/usr/bin/env python2

import rospy 
from std_msgs.msg import Bool 
from std_msgs.msg import UInt16

class RobotInitialisation(object): 
    def __init__(self): 
        rospy.init_node("robot_initialisation", anonymous=True)
        self.node_name = "merlion_init"
        self.init_states = [
            {
                "name": "ssh_command_sent", 
                "start_state": False
            },
            {
                "name": "motor", 
                "start_state": False
            }, 
            {
                "name": "imu", 
                "start_state": True
            }, 
            {
                "name": "hydrophones", 
                "start_state": True
            }, 
            {
                "name": "front_camera", 
                "start_state": True
            }, 
            {
                "name": "bottom_camera", 
                "start_state": True
            }, 
        ]
        
        def set_state(robot_object, init_state):
            name = init_state["name"]
            start_state = init_state["start_state"]
            setattr(self, f"{name}_active",start_state)  
            setattr(self, f"{name}_init_sub", rospy.Subscriber(f"/{self.node_name}/{name}_active", Bool, lambda msg: self.set_init_state(msg, name)))
        
        for init_state in self.init_states: 
            set_state(self, init_state)

        self.robot_state = 0
        self.robot_init_pub = rospy.Publisher(f"/{self.node_name}/robot_state",UInt16,queue_size=10)
    
    def set_init_state(self, state, name): 
        setattr(self,f"{name}_active",state)
        self.publish_init_state() 


    def publish_init_state(self): 
        all_states_active = True 
        for init_state in self.init_states: 
            name = init_state["name"]
            if not getattr(self, f"{name}_active"): 
                print(f"{name} not active")
                all_states_active = False 

        if all_states_active:
            self.robot_state = 1
        
        message = UInt16() 
        message.data = self.robot_state

        self.robot_init_pub.publish(message) 

if __name__ == "__main__": 
    robot = RobotInitialisation() 
    robot.publish_init_state()
    rospy.spin() 
