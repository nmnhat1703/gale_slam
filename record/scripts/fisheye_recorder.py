#!/usr/bin/env python3

import os
from datetime import datetime

import cv2
import rospy


def _camera_pipeline(sensor_id, width, height, fps, flip_method):
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM),width={width},height={height},framerate={int(fps)}/1 ! "
        f"nvvidconv flip-method={flip_method} ! "
        "video/x-raw,format=BGRx ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=true sync=false"
    )


def main():
    rospy.init_node("fisheye_recorder")

    sensor_id = int(rospy.get_param("~sensor_id", 0))
    output_dir = rospy.get_param("~output_dir", "/home/gale/gale_ws/src/record")
    width = int(rospy.get_param("~width", 640))
    height = int(rospy.get_param("~height", 480))
    fps = float(rospy.get_param("~fps", 30.0))
    flip_method = int(rospy.get_param("~flip_method", 2))
    output_prefix = rospy.get_param("~output_prefix", "fisheye")
    fourcc_name = rospy.get_param("~fourcc", "MJPG")

    os.makedirs(output_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_path = os.path.join(output_dir, f"{output_prefix}_{timestamp}.avi")

    pipeline = _camera_pipeline(sensor_id, width, height, fps, flip_method)
    rospy.loginfo("Opening fisheye camera pipeline: %s", pipeline)

    cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        rospy.logerr("Could not open fisheye camera pipeline")
        return

    writer_fourcc = cv2.VideoWriter_fourcc(*fourcc_name[:4])
    writer = cv2.VideoWriter(
        output_path,
        writer_fourcc,
        fps,
        (width, height),
    )
    if not writer.isOpened():
        cap.release()
        rospy.logerr("Could not open output video file: %s", output_path)
        return

    rospy.loginfo(
        "Recording fisheye camera sensor %d to %s at %dx%d %.2f FPS with flip-method=%d",
        sensor_id,
        output_path,
        width,
        height,
        fps,
        flip_method,
    )

    try:
        while not rospy.is_shutdown():
            ok, frame = cap.read()
            if not ok:
                rospy.logwarn_throttle(5.0, "No frame received from fisheye camera")
                rospy.sleep(0.01)
                continue

            if frame.shape[1] != width or frame.shape[0] != height:
                frame = cv2.resize(frame, (width, height))
            writer.write(frame)
    finally:
        writer.release()
        cap.release()
        rospy.loginfo("Closed fisheye recording: %s", output_path)


if __name__ == "__main__":
    main()
