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
ros2 bag info ./2026_03_24_E2E
```

### Extract Odometry Data

Extract odometry messages from the rosbag into individual JSON files organized by episode and timestep:

```bash
# Inside the container
python3 extract_odometry.py ./data
```

This creates:
```
data/
  episode_0/
    step_000000/
      odometry.json
    step_000001/
      odometry.json
    ...
```

