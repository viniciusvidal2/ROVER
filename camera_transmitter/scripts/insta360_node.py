#!/usr/bin/env python3

import rospy
import cv2
import os
import roslib
from flask import Flask, Response

app = Flask(__name__)

# General parameters
WIDTH = 1920
HEIGHT = 1080
FPS = 5

def transmit_insta360_http():
    cap = cv2.VideoCapture(0)

    if cap.isOpened():
        # Set the desired frame rate
        original_fps = 30
        frames_to_skip = int(original_fps/FPS) - 1

        frame_count = 0
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if not ret:
                continue
            frame_count += 1

            # Control the frame rate
            if frame_count == frames_to_skip + 1:
                frame_count = 0
                frame = cv2.resize(frame, (WIDTH, 2*HEIGHT))
                # Send the frame with the server
                _, buffer = cv2.imencode('.jpg', frame)
                buffer_bytes = buffer.tobytes()
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + buffer_bytes + b'\r\n')

        cap.release()
    else:
        # Simply display an image to inform the malfunctioning or abscent camera
        package_path = roslib.packages.get_pkg_dir("camera_transmitter")
        image = cv2.imread(os.path.join(
            package_path, "resources", "no_camera.png"))

        # Send the bad camera image from time to time
        while not rospy.is_shutdown():
            image = cv2.resize(image, (WIDTH, 2*HEIGHT))
            rospy.sleep(1)
            _, buffer = cv2.imencode('.jpg', frame)
            buffer_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer_bytes + b'\r\n')


@app.route('/video_feed')
def video_feed():
    return Response(transmit_insta360_http(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    try:
        rospy.init_node('insta360_node', anonymous=False)

        WIDTH = rospy.get_param('~width', 1920)
        HEIGHT = rospy.get_param('~height', 1080)
        FPS = rospy.get_param('~fps', 5)

        app.run(host='0.0.0.0', port=1234)

    except rospy.ROSInterruptException:
        print("ROS node terminated.")
