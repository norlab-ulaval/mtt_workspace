#!/usr/bin/env python3
import argparse
import os
import sys
import rosbag2_py
import sensor_msgs_py.point_cloud2 as pc2
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

def extract_frame(bag_path, topic, output_file, frame_index=0, start_time_offset=0.0):
    if not os.path.exists(bag_path):
        print(f"Error: Path {bag_path} does not exist")
        return

    # Store the original bag path to determine output directory
    session_dir = os.path.abspath(bag_path)
    if os.path.isfile(session_dir):
        session_dir = os.path.dirname(session_dir)

    # If it's a directory, look for metadata.yaml
    if os.path.isdir(bag_path):
        if not os.path.exists(os.path.join(bag_path, "metadata.yaml")):
            # Check for 'bag' subdirectory
            if os.path.exists(os.path.join(bag_path, "bag", "metadata.yaml")):
                bag_path = os.path.join(bag_path, "bag")
                # If we found it in /bag, the session dir is one level up
                if session_dir.endswith("/bag") or session_dir.endswith("/bag/"):
                    session_dir = os.path.dirname(session_dir)
            else:
                print(f"Error: {bag_path} is a directory but no metadata.yaml found")
                return

    # If output_file is just a name (no directory), put it in the session_dir
    if not os.path.dirname(output_file):
        output_file = os.path.join(session_dir, output_file)
    
    # Ensure directory exists
    os.makedirs(os.path.dirname(os.path.abspath(output_file)), exist_ok=True)

    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='mcap')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr')

    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(storage_options, converter_options)
    except Exception as e:
        print(f"Error opening bag: {e}")
        return

    topic_types = reader.get_all_topics_and_types()
    type_map = {topic_types[i].name: topic_types[i].type for i in range(len(topic_types))}
    
    if topic not in type_map:
        print(f"Topic {topic} not found in bag")
        print("Available topics:")
        for t in type_map:
            if "PointCloud2" in type_map[t]:
                print(f"  {t}")
        return

    msg_type_name = type_map[topic]
    msg_type = get_message(msg_type_name)
    
    metadata = reader.get_metadata()
    bag_start_time = metadata.starting_time.nanoseconds
    
    target_time_ns = bag_start_time + int(start_time_offset * 1e9)
    
    # Seek to time if offset > 0
    if start_time_offset > 0:
        reader.seek(target_time_ns)

    count = 0
    found = False
    while reader.has_next():
        (topic_name, data, t) = reader.read_next()
        if topic_name == topic:
            if count >= frame_index:
                msg = deserialize_message(data, msg_type)
                
                # Get fields
                field_names = [f.name for f in msg.fields]
                print(f"Fields available: {field_names}")
                
                # Check for intensity or reflectivity
                intensity_field = None
                if 'intensity' in field_names:
                    intensity_field = 'intensity'
                elif 'reflectivity' in field_names:
                    intensity_field = 'reflectivity'
                
                extract_fields = ["x", "y", "z"]
                if intensity_field:
                    extract_fields.append(intensity_field)
                
                points = pc2.read_points(msg, field_names=extract_fields, skip_nans=True)
                points_list = list(points)
                
                # Save to PLY
                with open(output_file, 'w') as f:
                    f.write("ply\n")
                    f.write("format ascii 1.0\n")
                    f.write(f"element vertex {len(points_list)}\n")
                    f.write("property float x\n")
                    f.write("property float y\n")
                    f.write("property float z\n")
                    if intensity_field:
                        f.write(f"property float {intensity_field}\n")
                    f.write("end_header\n")
                    for p in points_list:
                        line = " ".join(map(str, p))
                        f.write(f"{line}\n")
                
                print(f"Frame extracted at t={(t - bag_start_time)/1e9:.3f}s (offset {start_time_offset}s, index {count})")
                print(f"Saved {len(points_list)} points to {output_file}")
                found = True
                break
            count += 1
    
    if not found:
        print(f"Could not find frame {frame_index} after time offset {start_time_offset}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Extract a PointCloud2 frame from a ROS 2 bag")
    parser.add_argument("bag_path", help="Path to the bag file or directory")
    parser.add_argument("--topic", default="/rsairy_ns/points", help="Topic name")
    parser.add_argument("--output", default="frame.ply", help="Output PLY file")
    parser.add_argument("--index", type=int, default=0, help="Frame index (after time offset)")
    parser.add_argument("--time", type=float, default=0.0, help="Time offset in seconds from start")
    
    args = parser.parse_args()
    extract_frame(args.bag_path, args.topic, args.output, args.index, args.time)
