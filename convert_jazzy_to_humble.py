#!/usr/bin/env python3
"""
Convert ROS2 Jazzy rosbag metadata.yaml to Humble format.

This script reads a Jazzy format metadata.yaml and converts it to Humble format by:
1. Converting offered_qos_profiles from YAML list to escaped string
2. Mapping QoS enum values (e.g., unknown -> 3, automatic -> 1)
3. Removing Jazzy-specific fields (type_description_hash, custom_data, ros_distro)

The original file is NOT modified. Output is written to 'metadata_humble.yaml'.
"""

import yaml
import sys
from pathlib import Path


# QoS enum mappings from Jazzy to Humble
QOS_HISTORY_MAP = {
    "unknown": "3",
    "keep_last": "0",
    "keep_all": "1",
}

QOS_RELIABILITY_MAP = {
    "unknown": "3",
    "reliable": "1",
    "best_effort": "0",
}

QOS_DURABILITY_MAP = {
    "unknown": "3",
    "volatile": "2",
    "transient_local": "0",
    "transient": "1",
}

QOS_LIVELINESS_MAP = {
    "unknown": "3",
    "automatic": "1",
    "manual_by_node": "0",
    "manual_by_topic": "2",
}


def convert_qos_profile_to_string(profile):
    """Convert a QoS profile dict to Humble's escaped string format."""
    # Map the enum values
    converted = {
        "history": QOS_HISTORY_MAP.get(profile.get("history", "unknown"), "3"),
        "depth": profile.get("depth", 0),
        "reliability": QOS_RELIABILITY_MAP.get(profile.get("reliability", "unknown"), "3"),
        "durability": QOS_DURABILITY_MAP.get(profile.get("durability", "unknown"), "3"),
        "deadline": profile.get("deadline", {"sec": 9223372036, "nsec": 854775807}),
        "lifespan": profile.get("lifespan", {"sec": 9223372036, "nsec": 854775807}),
        "liveliness": QOS_LIVELINESS_MAP.get(profile.get("liveliness", "unknown"), "3"),
        "liveliness_lease_duration": profile.get("liveliness_lease_duration", {"sec": 9223372036, "nsec": 854775807}),
        "avoid_ros_namespace_conventions": profile.get("avoid_ros_namespace_conventions", False),
    }
    
    # Format as YAML and escape it as a string
    qos_yaml = yaml.dump(converted, default_flow_style=False, sort_keys=False)
    # Remove trailing newline
    qos_yaml = qos_yaml.rstrip('\n')
    # Escape for string representation
    escaped = qos_yaml.replace('\n', '\\n').replace('"', '\\"')
    return escaped


def convert_metadata(input_file, output_file, backup_file):
    """Convert Jazzy metadata.yaml to Humble format."""
    print(f"Reading Jazzy metadata from: {input_file}")
    
    with open(input_file, 'r') as f:
        metadata = yaml.safe_load(f)
    
    # Process each topic to convert its QoS profiles
    topics = metadata['rosbag2_bagfile_information']['topics_with_message_count']
    for topic_entry in topics:
        topic_meta = topic_entry['topic_metadata']
        
        # Convert offered_qos_profiles from list to escaped string
        if 'offered_qos_profiles' in topic_meta:
            profiles = topic_meta['offered_qos_profiles']
            if isinstance(profiles, list) and len(profiles) > 0:
                # Use first profile (typically only one in Jazzy)
                profile_str = convert_qos_profile_to_string(profiles[0])
                topic_meta['offered_qos_profiles'] = profile_str
        
        # Remove Jazzy-specific fields
        topic_meta.pop('type_description_hash', None)
    
    # Remove Jazzy-specific top-level fields
    metadata['rosbag2_bagfile_information'].pop('custom_data', None)
    metadata['rosbag2_bagfile_information'].pop('ros_distro', None)
    
    # Backup original Jazzy metadata
    print(f"Backing up original Jazzy metadata to: {backup_file}")
    import shutil
    shutil.copy(input_file, backup_file)
    
    # Write converted metadata as metadata.yaml (replaces original)
    print(f"Writing Humble metadata to: {output_file}")
    with open(output_file, 'w') as f:
        yaml.dump(metadata, f, default_flow_style=False, sort_keys=False, allow_unicode=True)
    
    print(f"✓ Conversion complete!")
    print(f"  Original file backed up to: {backup_file}")
    print(f"  New file: {output_file} (Humble format)")


if __name__ == '__main__':
    script_dir = Path(__file__).parent
    bag_dir = script_dir / '2026_03_24_E2E'
    input_metadata = bag_dir / 'metadata.yaml'
    output_metadata = bag_dir / 'metadata.yaml'  # Will overwrite original
    backup_metadata = bag_dir / 'metadata_jazzy.yaml'
    
    if len(sys.argv) > 1:
        input_metadata = Path(sys.argv[1])
        output_metadata = input_metadata
        backup_metadata = input_metadata.parent / f"{input_metadata.stem}_jazzy{input_metadata.suffix}"
    
    if not input_metadata.exists():
        print(f"Error: Input file not found: {input_metadata}")
        sys.exit(1)
    
    try:
        convert_metadata(input_metadata, output_metadata, backup_metadata)
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)
