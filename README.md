# Holohover

## ROS2 Setup

* Install ROS2 Galactic (Ubuntu 20.04) by following the guide [here](https://docs.ros.org/en/galactic/Installation/Ubuntu-Install-Debians.html).

* Clone the content of the repo into `~/holohover_ws/src`
  ```
  cd ~/holohover_ws/src
  git clone --recursive https://github.com/RSchwan/holohover.git
  ```

* Source the ROS2 installation
  ```
  source /opt/ros/$ROS_DISTRO/setup.bash
  ```

* Update dependencies using rosdep
  ```
  sudo apt update && rosdep update
  rosdep install --from-path src --ignore-src -y
  ```

* Build the project and source it
  ```
  colcon build
  source install/local_setup.bash
  ```

## ESP32 Setup

### Install and set up ESP-IDF

* Make sure you have installed all [prerequisites](https://docs.espressif.com/projects/esp-idf/en/release-v4.3/esp32/get-started/index.html#step-1-install-prerequisites).

* Download ESP-IDF 4.3 (we assume here to install it in `~/esp/` but feel free to change the location):
    ```
    mkdir -p ~/esp
    cd ~/esp
    git clone -b release/v4.3 --recursive https://github.com/espressif/esp-idf.git
    ```

* Clone [micro-ROS component for ESP-IDF](https://github.com/micro-ROS/micro_ros_espidf_component) into the components folder of ESP-IDF:
    ```
    cd ~/esp/esp-idf/components
    git clone https://github.com/micro-ROS/micro_ros_espidf_component.git
    ```

* Setup ESP-IDF:
    ```
    ~/esp/esp-idf/install.sh
    ```

* Set up the environment variables (note the space after the point):
    ```
    . ~/esp/esp-idf/export.sh
    ```

* The component needs `colcon` and other Python 3 packages inside the IDF virtual environment in order to build micro-ROS packages:
    ```
    pip3 install catkin_pkg lark-parser empy colcon-common-extensions importlib-resources
    ```

### Configuring and building the project

* Set up the environment variables if not already done:
    ```
    . ~/esp/esp-idf/export.sh
    ```

* Set the target
    ```
    idf.py set-target esp32
    ```

* Set your micro-ROS configuration and WiFi credentials under `micro-ROS Settings` using
    ```
    idf.py menuconfig
    ```
    Under `micro-ROS Settings --->` set the `micro-ROS Agent IP` and `micro-ROS Agent Port`,  and under `micro-ROS Settings ---> WiFi Configuration --->` set up your WiFi credentials.
    
* Then build the project
    ```
    idf.py build
    ```

#### Error Handing

If you run into the error `FAILED: esp-idf/mbedtls/x509_crt_bundle`, run
```
idf.py menuconfig
```
navigate to `Component config  ---> mbedTLS  ---> Certificate Bundle  ---> Default certificate bundle options` and chose `Use only the most common certificates from the default bundles`.

After fixing compilation errors it may be helpful to clean and rebuild all the micro-ROS library:
```
idf.py clean-microros
```

### CLion setup

The project may also be directly build using cmake. To set up CLion follow the guide [here](https://www.jetbrains.com/help/clion/esp-idf.html#cmake-setup).

### Flashing and monitoring

To flash the compiled program onto the ESP32 run:
```
idf.py flash
```

You can monitor serial messages with:
```
idf.py monitor
```

### Running the micro-ROS agent

To connect micro-ROS running on the ESP32 with the remaining ROS2 network we have to start the micro-ROS agent:
```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```
