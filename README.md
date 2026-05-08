# convert_ros2_bag
This repository extracts information from a ros2 bag file generating a dataset.
## Setup

### Prerequisites
- Docker

### Installation with Docker

1. Build the Docker image:
```bash
docker build -t rosbag_converter .
```

2. Run the container with proper user permissions:
```bash
docker run -it -u $(id -u):$(id -g) -e USER=$(whoami) -v $(pwd):/workspace rosbag_converter /bin/bash
```


### Convert Jazzy to Humble

If your rosbag was recorded with ROS2 Jazzy, convert the metadata to Humble format first:

```bash
# Inside the container
python3 convert_jazzy_to_humble.py /path_to_ros2_bag
```

This script:
- Reads `rosbag_folder/metadata.yaml` (Jazzy format)
- Backs up original to `rosbag_folder/metadata_jazzy.yaml`
- Writes converted version to `rosbag_folder/metadata.yaml` (Humble format)

**Note:** The original metadata is safely backed up before overwriting.

### Check the ros2 bag topics

If you want to check the ros2 bag topics run:
```bash
# Inside the container
ros2 bag info /path_to_ros2_bag
```

### Extract Data

Extract multimodal sensor data from the rosbag at 10 Hz (0.1s time windows), organized by episode and timestep:

```bash
# Inside the container

# Extract all MCAP files (all data)
python3 extract_multimodal.py ./2026_03_24_E2E ./data

# Extract only the first file
python3 extract_multimodal.py ./2026_03_24_E2E ./data 1

# Extract the first file with max 50 timesteps 
python3 extract_multimodal.py ./2026_03_24_E2E ./data 1 50
```

**Arguments:**
- `bag_path`: Path to the rosbag directory (default: `./2026_03_24_E2E`)
- `output_dir`: Output directory for extracted data (default: `./data`)
- `max_files`: Maximum number of MCAP files to process (optional, default: all files)
- `max_timesteps`: Maximum number of timesteps per file (optional, default: all timesteps)

**Output Structure:**

Each episode (MCAP file) creates a directory with 0.1s timesteps:
```
data/
  episode_0/
    step_000000/
      odometry.json                  # Pose and twist data
      steering_angle.json            # Steering measurement
      image_cl.png                   # Front-left camera (demosaicked)
      image_fl.png                   # Front camera
      image_cr.png                   # Center-right camera
      image_rc.png                   # Rear-center camera
      image_fc.png                   # Front-center camera
      lidar_os0_metadata.json        # Sensor calibration data
      lidar_os0_packets.bin          # Raw lidar packets
      lidar_os1_metadata.json
      lidar_os1_packets.bin
      lidar_os2_metadata.json
      lidar_os2_packets.bin
    step_000001/
      ...
  episode_1/
    step_000000/
      ...
```

**Data Formats:**
- Images: PNG (color, demosaicked from Bayer pattern)
- Odometry: JSON with position (x,y,z) and orientation (quaternion) and velocity
- Steering: JSON with angle measurement
- Lidar: Metadata (JSON) + Raw packets (binary, can be decoded with Ouster SDK)

