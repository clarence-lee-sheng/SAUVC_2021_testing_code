#!/usr/bin/env python3

import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image 

import numpy as np

import json
import requests
import time 

class Camera: 
    def __init__(self, topic): 
        self.bridge = CvBridge() 
        self.camera_pub = rospy.Publisher(topic, Image, queue_size=10)


    def test_video(self):
        video_capture = cv2.VideoCapture(4)

        frame_width = int(video_capture.get(3))
        frame_height = int(video_capture.get(4))
        video_writer = cv2.VideoWriter('record.avi',cv2.VideoWriter_fourcc(*"MJPG"), 10, (frame_width, frame_height))

        while(True): 
            ret, frame = video_capture.read()
            video_writer.write(frame)
            cv2.imshow("frame", frame)
            
            if cv2.waitKey(100) & 0xFF == ord('q'): 
                break
        video_writer.release()
        video_capture.release()
        
        cv2.destroyAllWindows()

camera = Camera("front_camera")
camera.test_video() 


headers = {
    "Authorization": "Bearer ya29.a0ARrdaM83_HV29jlkLncdxSjrAa78nHSIio0SMbwEe6xAu4CsC1KTnxGXlX-UN7BTj0azZluLYkSNF7qwvHJzrjbUzFeTo86gQ4M8SHVJhkJuNwh7VAJB6P7cZkJ7x6fKkdbTG86j-1WF0ELyC9HgQmt8XRLQ"
}

para = {
    "name": "record.avi",
    "parents": ["1yiq7O8SEJ2mNQvNXEPrPIDHSWEmpRfzP"]
}

files = {
    'data': ('metadata', json.dumps(para), 'application/json; charset=UTF-8'),
    'file': open("record.avi", "rb")
}

# while True: 



refresh = requests.post(
    "https://accounts.google.com/o/oauth2/v2/auth",
    {"grant_type": "1//04u8ThmrRvlqSCgYIARAAGAQSNwF-L9Irrb9q1G55-6-22idQdlxgHbz6fwR8yRCLsskuECZBTkv7Sr7ypVUiCZKSr9xn8w6oNbg"}
)
print(refresh.status_code)

r = requests.post(
    "https://www.googleapis.com/upload/drive/v3/files?uploadType=multipart",
    headers=headers,
    files=files
)
print(r.status_code)

# if r.status_code == 200:
#     break

time.sleep(2)


# all_camera_idx_available = []

# for camera_idx in range(10):
#     cap = cv2.VideoCapture(camera_idx)
#     if cap.isOpened():
#         print("camera open", camera_idx)
#         # print (f'Camera index available: {camera_idx}')
#         all_camera_idx_available.append(camera_idx)
#         cap.release()