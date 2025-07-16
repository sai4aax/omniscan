# OmniScan Sonar Driver

This package provides a ROS 2 driver for the OmniScan sonar. It includes nodes for interfacing with the hardware, publishing sonar data, and converting it to point clouds for visualization.

## Getting Started

### Prerequisites

- ROS 2 (Foxy Fitzroy or later)
- `colcon`

### Installation

1.  **Create workspace**

    ```bash
    mkdir -p ~/sonar_ws/src 
    cd ~/sonar_ws/src
    ```

2.  **Clone the repository:**

    ```bash
    git clone https://github.com/sai4aax/omniscan.git
    ```

3.  **Build the packages:**

    ```bash
    cd ~/sonar_ws
    colcon build
    echo "source ~/sonar_ws/install/setup.bash"
    ```
4.  **Some tricks for fast development:**

    ```bash
    echo "source ~/sonar_ws/install/setup.bash" >> ~/.bashrc
    echo "alias bomni='cd ~/sonar_ws && colcon build && souce install/setup.bash'" >> ~/.bashrc
    echo "alias somni='source ~/sonar_ws/install/setup.bash'" >> ~/.bashrc
    ```

    if you made any changes to the code, use the below command before running the other nodes

    ```
    bomni
    ```


## Packages

This repository contains the following packages:

-   `omniscan_driver`: Contains the main driver nodes, including the hardware interface, a mock sensor for testing, and a point cloud converter.
-   `omniscan_msgs`: Defines the custom ROS messages used by the driver, specifically the `OsMonoProfile` message.

## Nodes

### `omniscan_driver`

-   **`sonar_node`**: The main driver node that communicates with the OmniScan sonar hardware via UDP.
-   **`mock_sensor`**: A node that simulates the OmniScan sonar, publishing mock data for testing and development.
-   **`sonar2points`**: A node that converts the raw `OsMonoProfile` messages from the sonar into `PointCloud2` messages, which can be visualized in RViz2.
-   **`tf_publish`**: A node for publishing transforms, for example to position the sonar relative to the robot's base frame.
-   **`omniscan_lifecycle`**: A lifecycle node for managing the driver's state. but still its not ready yet.

### `omniscan_msgs`

-   **`publish_os_mono_profile_test.py`**: A script for publishing test `OsMonoProfile` messages.

## Usage

### Running the Mock Sensor

Run this in all terminal or the above added line to the bashrc file will automatically source it everytime new terminal is opened

```bash
somni
```


### Running the Mock Sensor

To run the driver with the mock sensor and visualize the output in RViz2, follow these steps:

1.  **Launch the mock sensor:**
    
    terminal 1
    ```bash
    ros2 run omniscan_driver mock_sensor
    ```

2.  **Launch the sonar to point cloud converter:**

    terminal 2
    ```bash
    ros2 run omniscan_driver sonar2points --ros-args -p source_frame:=base_link -p target_frame:=sonar_link
    ```

3.  **Publish a static transform for visualization:**

    terminal 3
    ```bash
    ros2 run omniscan_driver dummy_tf 
    ```

4.  **Launch RViz2 and add a `PointCloud2` display, subscribing to the `/points` topic.**

### Running with Recorded Data

To run the driver with a recorded rosbag file:

1.  **Play the rosbag:**

    terminal 1
    ```bash
    ros2 bag play ~/sensors_ws/src/omniscan/ping_record
    ```

2.  **Launch the sonar to point cloud converter:**

    terminal 2
    ```bash
    ros2 run omniscan_driver sonar2points --ros-args -p source_frame:=base_link -p target_frame:=sonar_link
    ```

3.  **Publish a static transform for visualization:**

    terminal 3
    ```bash
    ros2 run omniscan_driver dummy_tf 
    ```

4.  **Launch RViz2 and add a `PointCloud2` display, subscribing to the `/points` topic.**

### Running the Sonar Node

To run the driver with the actual sonar hardware:

1.  **Launch the sonar node:**

    terminal 1
    ```bash
    ros2 run omniscan_driver sonar_node --udp 192.168.1.3:51200 # replace ip and port number with the actual ones
    ```

2.  **Launch the sonar to point cloud converter:**

    terminal 2
    ```bash
    ros2 run omniscan_driver sonar2points --ros-args -p source_frame:=base_link -p target_frame:=sonar_link
    ```

3.  **Publish a static transform for visualization:**

    terminal 3
    ```bash
    ros2 run omniscan_driver dummy_tf 
    ```

4.  **Launch RViz2 and add a `PointCloud2` display, subscribing to the `/points` topic.**

## Contributing

Contributions are welcome! Please open an issue or submit a pull request.

## License

This project is licensed under the TODO: License declaration.
