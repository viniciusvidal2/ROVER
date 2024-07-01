#!/usr/bin/env python3

import rospy
import cv2
import os
import roslib


def transmit_camera_hdmi(width: int, height: int, desired_fps: float):
    # Suppress OpenCV warnings
    os.environ['OPENCV_VIDEOIO_PRIORITY_MSMF'] = '0'
    os.environ['OPENCV_VIDEOIO_PRIORITY_GSTREAMER'] = '0'

    cap = cv2.VideoCapture(0)

    if cap.isOpened():
        # Set the desired frame rate
        original_fps = 30
        frames_to_skip = int(original_fps/desired_fps) - 1

        print("Desired frame rate:", desired_fps)
        print("Frames to skip:", frames_to_skip)

        # Create a custom window
        window_name = 'Vehicle Camera'
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(
            window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        frame_count = 0
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            frame_count += 1
            if not ret:
                continue

            # Control the frame rate
            if frame_count == frames_to_skip + 1:
                frame_count = 0
                frame = cv2.resize(frame, (width, height))
                frame = cv2.imshow(window_name, frame)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        cap.release()
    else:
        # Simply display an image to inform the malfunctioning or abscent camera
        package_path = roslib.packages.get_pkg_dir("camera_transmitter")
        image = cv2.imread(os.path.join(
            package_path, "resources", "no_camera.png"))

        # Create a custom window
        window_name = 'Bad Camera'
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(
            window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        while not rospy.is_shutdown():
            image = cv2.resize(image, (width, height))
            cv2.imshow(window_name, image)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            rospy.sleep(0.2)


if __name__ == '__main__':
    try:
        rospy.init_node('camera_transmitter', anonymous=False)

        width = rospy.get_param('~width', 1920)
        height = rospy.get_param('~height', 1080)
        fps = rospy.get_param('~fps', 5)

        transmit_camera_hdmi(width, height, fps)

    except rospy.ROSInterruptException:
        print("ROS node terminated.")
