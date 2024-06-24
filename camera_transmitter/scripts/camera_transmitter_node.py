#!/usr/bin/env python3

import rospy
import cv2

def transmit_camera_hdmi(width: int, height: int, desired_fps: float):
    cap = cv2.VideoCapture(0)
    
    # Set the desired frame rate
    original_fps = 30
    frames_to_skip = int(original_fps/desired_fps) - 1

    print("Desired frame rate:", desired_fps)
    print("Frames to skip:", frames_to_skip)

    # Create a custom window
    window_name = 'Vehicle Camera'
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
            
            # Resize the image to the desired size
            frame = cv2.resize(frame, (width, height))
            
            # Display frame
            frame = cv2.imshow(window_name, frame)
            
            # Check for quit event
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()


if __name__ == '__main__':
    try:
        rospy.init_node('camera_transmitter', anonymous=False)
    
        width = rospy.get_param('~width', 1920)
        height = rospy.get_param('~height', 1080)
        fps = rospy.get_param('~fps', 5)
        
        transmit_camera_hdmi(width, height, fps)
        
    except rospy.ROSInterruptException:
        print("ROS node terminated.")
