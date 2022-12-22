from rosbags.rosbag1 import Reader as Reader1
from rosbags.rosbag2 import Reader as Reader2
from rosbags.serde import deserialize_cdr, ros1_to_cdr
from rosbags.typesys import get_types_from_msg, register_types

import csv
import argparse

# tag detection message definition
tag_detection_msg = """
int32[] id
float64[] size
geometry_msgs/PoseWithCovarianceStamped pose
"""
register_types(get_types_from_msg(tag_detection_msg, 'apriltag_ros/msg/AprilTagDetection'))

# image detection message definition
image_detection_msg = """
std_msgs/Header header
AprilTagDetection[] detections
"""
register_types(get_types_from_msg(image_detection_msg, 'apriltag_ros/msg/AprilTagDetectionArray'))


def append_det_to_array(array, det_msg, timestamp):
    # Extract the tag data
    image_id = timestamp
    tag_id = det_msg.id
    tag_size = det_msg.size
    t_x = det_msg.pose.pose.pose.position.x
    t_y = det_msg.pose.pose.pose.position.y
    t_z = det_msg.pose.pose.pose.position.z
    q_x = det_msg.pose.pose.pose.orientation.x
    q_y = det_msg.pose.pose.pose.orientation.y
    q_z = det_msg.pose.pose.pose.orientation.z
    q_w = det_msg.pose.pose.pose.orientation.w
    
    # Append the data to the list
    array.append([image_id, tag_id, tag_size, t_x, t_y, t_z, q_x, q_y, q_z, q_w])

def parse_rosbag2_to_csv(rosbag_folder, csv_location):
    # initialize empty detection array
    detection_array = []
    # create reader instance and open for reading
    with Reader2(rosbag_folder) as reader:
        filt_connections = [x for x in reader.connections if x.topic == '/tag_detections']
        for connection, timestamp, rawdata in reader.messages(connections=filt_connections):
            tag_detection_msg = deserialize_cdr(rawdata, connection.msgtype)
            for det_msg in tag_detection_msg:
                if len(det_msg.id)>1:
                    # Can only use standalone tag detections for calibration!
                    # The math allows for bundles too (e.g. bundle composed of
                    # bundles) but the code does not, and it's not that useful
                    # anyway
                    print('Skipping tag bundle detections')
                    continue
                append_det_to_array(detection_array, det_msg, tag_detection_msg.header.timestamp)

        #write to csv
        with open(csv_location, "w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerows(detection_array)

def parse_rosbag1_to_csv(rosbag_folder, csv_location):
    # initialize empty detection array
    detection_array = []
    # create reader instance and open for reading
    with Reader1(rosbag_folder) as reader:
        filt_connections = [x for x in reader.connections if x.topic == '/tag_detections']
        for connection, timestamp, rawdata in reader.messages(connections=filt_connections):
            tag_detection_msg = deserialize_cdr(ros1_to_cdr(rawdata, connection.msgtype), connection.msgtype)
            for det_msg in tag_detection_msg:
                if len(det_msg.id)>1:
                    # Can only use standalone tag detections for calibration!
                    # The math allows for bundles too (e.g. bundle composed of
                    # bundles) but the code does not, and it's not that useful
                    # anyway
                    print('Skipping tag bundle detections')
                    continue
                append_det_to_array(detection_array, det_msg, tag_detection_msg.header.timestamp)

        #write to csv
        with open(csv_location, "w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerows(detection_array)

def main(args=None):
    parser = argparse.ArgumentParser(description='Script to compute relative poses of tags in master tag frame')
    parser.add_argument('-i', '--rosbag_input', help='path to input bag (folder if ros2, .bag file if ros1)', required=True, type=str)
    parser.add_argument('-o', '--csv_path_output', help='path to output bag', required=True, type=str)
    parser.add_argument('-r', '--ros', help='rosbag version (default = 1)', required=False, type=int, default=2)
    args = parser.parse_args()
    if args.ros == 2:
        parse_rosbag2_to_csv(args.rosbag_input, args.csv_path_output)
    else:
        parse_rosbag1_to_csv(args.rosbag_input, args.csv_path_output)


if __name__ == "__main__":
    main()