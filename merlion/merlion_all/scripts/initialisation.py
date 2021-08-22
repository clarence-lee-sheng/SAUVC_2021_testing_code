#!/usr/bin/env python3

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

        # self.motor_active = False 
        # self.motor_init_sub = rospy.Subscriber("/merlion_init/motor_active", Bool, self.set_motor_active)

        # self.imu_active = True 
        # self.imu_init_sub = rospy.Subscriber("/merlion_init/imu_active", self.set_imu_active)

        # self.hydrophones_received = True 
        # self.hydrophones_init_sub = rospy.Subscriber("/merlion_init/hydrophones_active", self.set_hydrophones_active)

        # self.front_camera_active = True
        # self.front_camera_init_sub = rospy.Subscriber("/merlion_init/front_camera_active", self.set_front_camera_active)

        # self.bottom_camera_active = True 
        # self.bottom_camera_init_sub = rospy.Subscriber("/merlion_init/bottom_camera_active", self.set_bottom_camera_active)
    
    def set_init_state(self, state, name): 
        print(state.data)
        print(name)
        setattr(self,f"{name}_active",state)
        print(getattr(self, f"{name}_active"))
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