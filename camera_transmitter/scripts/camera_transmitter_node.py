#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

def transmit_camera_hdmi(width: int, height: int, desired_fps: float):
    cap = cv2.VideoCapture(0)
    
    # Set the desired frame rate
    if desired_fps == 0:
        desired_fps = cap.get(cv2.CAP_PROP_FPS)
    original_fps = cap.get(cv2.CAP_PROP_FPS)
    frames_to_skip = int(original_fps/desired_fps) - 1
    cap.set(cv2.CAP_PROP_FPS, desired_fps)
    if original_fps == 0:
        frames_to_skip = 0
        print("Warning: Frame rate not available.")
    if desired_fps == cap.get(cv2.CAP_PROP_FPS):
        frames_to_skip = 0
        print("Frame rate set successfully.")
    print("Frame rate:", original_fps)
    print("Desired frame rate:", desired_fps)
    print("Frames to skip:", frames_to_skip)
    
    # Create a custom window
    window_name = 'Robot Camera'
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

    frame_count = 0
    while not rospy.is_shutdown():
        ret, frame = cap.read()
        frame_count += 1
        if not ret:
            continue
        
        # Control the frame rate
        if frame_count == frames_to_skip + 1:
            frame_count = 0
            # Display frame
            cv2.imshow(window_name, frame)
            # Check for quit event
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()


if __name__ == '__main__':
    try:
        rospy.init_node('camera_transmitter', anonymous=False)
    
        width = rospy.get_param('~width', 640)
        height = rospy.get_param('~height', 480)
        fps = rospy.get_param('~fps', 10)
        
        transmit_camera_hdmi(width, height, fps)
        
    except rospy.ROSInterruptException:
        print("ROS node terminated.")
    