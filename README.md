# Holohover

## ROS2 Setup

* Install ROS2 Humble (Ubuntu 22.04) by following the guide [here](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

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

### CLion setup (Optional)

The project may also be directly build using cmake. To set up CLion follow the guide [here](https://www.jetbrains.com/help/clion/esp-idf.html#cmake-setup).

### Configuring the project

* Cd into the esp32 project and set up the environment variables if not already done:
    ```
    cd ~/holohover_ws/src/holohover/esp32
    . ~/esp/esp-idf/export.sh
    ```

* Set the target
    ```
    idf.py set-target esp32
    ```
There are two options for the transport layer. After changing the transportation layer you have to clean the micro-ROS build with
```
idf.py clean-microros
```

#### Bluetooth Transport Layer

* Set `"-DRMW_UXRCE_TRANSPORT=custom"` in the file `app-colcon.meta`.

* Open the micro-ROS configuration
    ```
    idf.py menuconfig
    ```

* Navigate to `Component config ---> Bluetooth --->` and enable `Bluetooth`.

* Navigate to `Component config ---> Bluetooth ---> Bluetooth controller(ESP32 Dual Mode Bluetooth) ---> Bluetooth controller mode (BR/EDR/BLE/DUALMODE) (BR/EDR Only) --->` and select `BR/EDR Only`.

* Navigate to `Component config ---> Bluetooth ---> Bluedroid Options --->` and enable `Classic Bluetooth` and `Classic Bluetooth -> SPP`.

* Quit and save.

#### WIFI Transport Layer

* Set `"-DRMW_UXRCE_TRANSPORT=udp"` in the file `app-colcon.meta`.

* Open the micro-ROS configuration
    ```
    idf.py menuconfig
    ```
* Navigate to `micro-ROS Settings --->` and set the `micro-ROS Agent IP` and `micro-ROS Agent Port`.

* Navigate to `micro-ROS Settings ---> WiFi Configuration --->` and set up your WiFi credentials.

* Quit and save.

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

### Building, flashing, and monitoring

To build the project run:
```
idf.py build
```

To flash the compiled program onto the ESP32 run:
```
idf.py flash
```

You can monitor serial messages with:
```
idf.py monitor
```

### Connect the ESP32 to your ROS network with micro-ROS agent

#### Building the micro-ROS agent

To build the micro-ROS agent run the following:
```
cd ~/holohover_ws/
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh
```

#### Running the micro-ROS agent

To connect micro-ROS running on the ESP32 with the remaining ROS2 network we have to start the micro-ROS agent.

To use the Bluetooth transport layer run:
```
ros2 run micro_ros_agent micro_ros_agent serial -D /dev/tty.ESP32-ESP32SPP
```
where `/dev/tty.ESP32-ESP32SPP` has to be changed to match the bluetooth input which can be found by running `ls /dev`.

To use the WIFI transport layer run:
```
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```
