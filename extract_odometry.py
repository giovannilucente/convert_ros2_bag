#!/usr/bin/env python3
"""Extract odometry data from first MCAP file to JSON."""

import json
import sys
from pathlib import Path
from tqdm import tqdm

import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry


def extract_odometry(bag_path, output_dir, episode=0):
    """Extract odometry from first MCAP file to JSON, one file per timestep."""
    bag_path = Path(bag_path)
    output_dir = Path(output_dir)
    
    episode_dir = output_dir / f"episode_{episode}"
    episode_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"Reading from: {bag_path}")
    print(f"Output directory: {episode_dir}")
    
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions()
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    metadata = reader.get_metadata()
    print(f"Total messages: {metadata.message_count}")
    print(f"Files to process: {metadata.relative_file_paths[:1]}")
    
    topic_filter = "/ngc/sensors/gnss/pwrpak7/raw/odom"
    step_count = 0
    
    print(f"Extracting from: {topic_filter}")
    
    with tqdm(total=metadata.message_count, desc="Processing") as pbar:
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            pbar.update(1)
            
            if topic == topic_filter:
                msg = deserialize_message(data, Odometry)
                
                # Create step folder
                step_dir = episode_dir / f"step_{step_count:06d}"
                step_dir.mkdir(parents=True, exist_ok=True)
                odometry_file = step_dir / "odometry.json"
                
                # Create individual message dict
                odom_dict = {
                    "timestamp": timestamp,
                    "header": {
                        "stamp": {
                            "secs": msg.header.stamp.sec,
                            "nsecs": msg.header.stamp.nanosec,
                        },
                        "frame_id": msg.header.frame_id,
                    },
                    "child_frame_id": msg.child_frame_id,
                    "pose": {
                        "position": {
                            "x": float(msg.pose.pose.position.x),
                            "y": float(msg.pose.pose.position.y),
                            "z": float(msg.pose.pose.position.z),
                        },
                        "orientation": {
                            "x": float(msg.pose.pose.orientation.x),
                            "y": float(msg.pose.pose.orientation.y),
                            "z": float(msg.pose.pose.orientation.z),
                            "w": float(msg.pose.pose.orientation.w),
                        },
                    },
                    "twist": {
                        "linear": {
                            "x": float(msg.twist.twist.linear.x),
                            "y": float(msg.twist.twist.linear.y),
                            "z": float(msg.twist.twist.linear.z),
                        },
                        "angular": {
                            "x": float(msg.twist.twist.angular.x),
                            "y": float(msg.twist.twist.angular.y),
                            "z": float(msg.twist.twist.angular.z),
                        },
                    },
                }
                
                # Write individual JSON file
                with open(odometry_file, 'w') as f:
                    json.dump(odom_dict, f, indent=2)
                
                step_count += 1
    
    print(f"✓ Extracted {step_count} messages to: {episode_dir}")


if __name__ == '__main__':
    bag_path = Path(__file__).parent / '2026_03_24_E2E'
    output_dir = Path(__file__).parent / 'data'
    
    if len(sys.argv) > 1:
        output_dir = Path(sys.argv[1])
    
    extract_odometry(bag_path, output_dir)
