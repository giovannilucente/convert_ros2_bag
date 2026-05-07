#!/usr/bin/env python3
"""Extract odometry and images from rosbag at 10 Hz (0.1s intervals)."""

import json
import cv2
import numpy as np
from pathlib import Path
from collections import defaultdict
from tqdm import tqdm

import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Float32
from cv_bridge import CvBridge


def extract_multimodal_data(bag_path, output_dir, episode=0, max_files=1):
    """
    Extract odometry and images grouped by 0.1s time windows.
    """
    bag_path = Path(bag_path)
    output_dir = Path(output_dir)
    
    episode_dir = output_dir / f"episode_{episode}"
    episode_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"Reading from: {bag_path}")
    print(f"Output directory: {episode_dir}")
    print(f"Processing first {max_files} MCAP file(s) ...")
    
    storage_options = rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions()
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    metadata = reader.get_metadata()
    print(f"Total messages in bag: {metadata.message_count}")
    
    # Image topic to short name mapping
    image_topics = {
        "/ngc/sensors/camera/cl/mako223/wide/raw/image_raw": "image_cl",
        "/ngc/sensors/camera/fl/mako223/tele/raw/image_raw": "image_fl",
        "/ngc/sensors/camera/cr/mako223/wide/raw/image_raw": "image_cr",
        "/ngc/sensors/camera/rc/mako223/wide/raw/image_raw": "image_rc",
        "/ngc/sensors/camera/fc/mako223/wide/raw/image_raw": "image_fc",
    }
    
    odom_topic = "/ngc/sensors/gnss/pwrpak7/raw/odom"
    steering_topic = "/ego_vehicle/VEH/steering_angle_measured"
    time_window = 100_000_000  # 0.1s in nanoseconds
    
    # Group messages by time window
    time_buckets = defaultdict(lambda: {"odom": None, "images": {}, "steering_angle": None})
    first_timestamp = None
    
    with tqdm(total=metadata.message_count, desc="Reading") as pbar:
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            pbar.update(1)
            
            if first_timestamp is None:
                first_timestamp = timestamp
            
            # Calculate bucket index (0.1s windows)
            bucket_index = (timestamp - first_timestamp) // time_window
            
            if topic == odom_topic:
                msg = deserialize_message(data, Odometry)
                odom_dict = {
                    "timestamp": timestamp,
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
                time_buckets[bucket_index]["odom"] = odom_dict
            
            elif topic in image_topics:
                msg = deserialize_message(data, Image)
                bridge = CvBridge()
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                image_name = image_topics[topic]
                time_buckets[bucket_index]["images"][image_name] = cv_image
            
            elif topic == steering_topic:
                msg = deserialize_message(data, Float32)
                time_buckets[bucket_index]["steering_angle"] = float(msg.data)
            
            # Rough estimate: stop after processing enough from first file
            if bucket_index > 500:
                break
    
    print(f"Writing data for {len(time_buckets)} timesteps...")
    
    # Write each timestep
    for step_idx, (bucket_idx, data) in enumerate(sorted(time_buckets.items())):
        step_dir = episode_dir / f"step_{step_idx:06d}"
        step_dir.mkdir(parents=True, exist_ok=True)
        
        # Write odometry if available
        if data["odom"] is not None:
            odom_file = step_dir / "odometry.json"
            with open(odom_file, 'w') as f:
                json.dump(data["odom"], f, indent=2)
        
        # Write steering angle if available
        if data["steering_angle"] is not None:
            steering_file = step_dir / "steering_angle.json"
            with open(steering_file, 'w') as f:
                json.dump({"steering_angle": data["steering_angle"]}, f, indent=2)
        
        # Write images if available
        for image_name, cv_image in data["images"].items():
            img_file = step_dir / f"{image_name}.png"
            cv2.imwrite(str(img_file), cv_image)
    
    print(f"✓ Complete!")
    print(f"  Timesteps: {len(time_buckets)}")
    print(f"  Output: {episode_dir}")


if __name__ == '__main__':
    import sys
    
    script_dir = Path(__file__).parent
    bag_path = script_dir / '2026_03_24_E2E'
    output_dir = script_dir / 'data'
    
    if len(sys.argv) > 1:
        output_dir = Path(sys.argv[1])
    
    
    extract_multimodal_data(bag_path, output_dir, episode=0, max_files=1)
    
