#!/usr/bin/env python3
"""Extract odometry, images, and lidar from rosbag at 10 Hz (0.1s intervals)."""

import json
from logging import info
import cv2
import numpy as np
from pathlib import Path
from collections import defaultdict
from tqdm import tqdm

import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from std_msgs.msg import Float32, String
from cv_bridge import CvBridge
from ouster_sensor_msgs.msg import PacketMsg


def extract_multimodal_data(bag_path, output_dir, max_files=None, max_timesteps=None):
    """
    Extract odometry and images grouped by 0.1s time windows.
    
    Args:
        bag_path: Path to the rosbag directory
        output_dir: Output directory for extracted data
        max_files: Maximum number of MCAP files to process (None = all files)
        max_timesteps: Maximum number of timesteps to save per file (None = no limit)
    """
    bag_path = Path(bag_path)
    output_dir = Path(output_dir)
    
    # Find all MCAP files in the bag directory
    mcap_files = sorted(bag_path.glob("rosbag2_*.mcap"))
    if not mcap_files:
        print(f"No MCAP files found in {bag_path}")
        return
    
    if max_files is not None:
        mcap_files = mcap_files[:max_files]
    
    print(f"Found {len(mcap_files)} MCAP file(s)")
    
    # Process each file with its own episode number
    for episode, mcap_file in enumerate(mcap_files):
        _process_episode(mcap_file, output_dir, episode, max_timesteps)


def _process_episode(mcap_file, output_dir, episode, max_timesteps):
    """Process a single MCAP file as an episode."""
    
    episode_dir = output_dir / f"episode_{episode}"
    episode_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"\n[Episode {episode}] Reading from: {mcap_file}")
    print(f"Output directory: {episode_dir}")
    
    storage_options = rosbag2_py.StorageOptions(uri=str(mcap_file.parent), storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions()
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    
    metadata = reader.get_metadata()
    print(f"Total messages in file: {metadata.message_count}")
    
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
    
    # Lidar topics
    lidar_topics = {
        "/ngc/sensors/lidar/fr/os0/os_driver/raw/lidar_packets": "os0",
        "/ngc/sensors/lidar/fl/os2/os_driver/raw/lidar_packets": "os2",
        "/ngc/sensors/lidar/rl/os1/os_driver/raw/lidar_packets": "os1",
    }
    
    lidar_metadata_topics = {
        "/ngc/sensors/lidar/fr/os0/os_driver/raw/metadata": "os0",
        "/ngc/sensors/lidar/fl/os2/os_driver/raw/metadata": "os2",
        "/ngc/sensors/lidar/rl/os1/os_driver/raw/metadata": "os1",
    }
    
    time_window = 100_000_000  # 0.1s in nanoseconds
    
    # Store metadata globally (same for all timesteps)
    lidar_metadata_store = {}
    
    # Group messages by time window
    time_buckets = defaultdict(lambda: {
        "odom": None, 
        "images": {}, 
        "steering_angle": None,
        "lidar_packets": defaultdict(list),  # {lidar_name: [packets]}
        "lidar_metadata": {}  # {lidar_name: metadata_json}
    })
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
                cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
                image_name = image_topics[topic]
                time_buckets[bucket_index]["images"][image_name] = cv_image
            
            elif topic == steering_topic:
                msg = deserialize_message(data, Float32)
                time_buckets[bucket_index]["steering_angle"] = float(msg.data)
            
            elif topic in lidar_metadata_topics:
                msg = deserialize_message(data, String)
                lidar_name = lidar_metadata_topics[topic]
                lidar_metadata_store[lidar_name] = msg.data
            
            elif topic in lidar_topics:
                lidar_name = lidar_topics[topic]
                time_buckets[bucket_index]["lidar_packets"][lidar_name].append(data)
            
            # Stop if max timesteps reached
            if max_timesteps is not None and bucket_index >= max_timesteps:
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
        
        # Write lidar data if available
        for lidar_name, packets in data["lidar_packets"].items():
            if packets and lidar_name in lidar_metadata_store:
                try:
                    # Save metadata as JSON
                    metadata_str = lidar_metadata_store[lidar_name]
                    metadata_file = step_dir / f"lidar_{lidar_name}_metadata.json"
                    with open(metadata_file, 'w') as f:
                        f.write(metadata_str)
                    
                    # Save raw packets as binary file 
                    packets_file = step_dir / f"lidar_{lidar_name}_packets.bin"
                    with open(packets_file, 'wb') as f:
                        for packet_data in packets:
                            packet_msg = deserialize_message(packet_data, PacketMsg)
                            f.write(bytes(packet_msg.buf))
                    
                except Exception as e:
                    print(f"Warning: Failed to save lidar {lidar_name}: {e}")
                
        
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
    max_files = None
    max_timesteps = None
    
    if len(sys.argv) > 1:
        bag_path = Path(sys.argv[1])
    
    if len(sys.argv) > 2:
        output_dir = Path(sys.argv[2])
    
    if len(sys.argv) > 3:
        max_files = int(sys.argv[3])
    
    if len(sys.argv) > 4:
        max_timesteps = int(sys.argv[4])
    
    extract_multimodal_data(bag_path, output_dir, max_files=max_files, max_timesteps=max_timesteps)
    
