# Holohover Firmware

## Firmware Setup

Again, there are two options to build the firmware for the ESP: using Docker or installing the ESP-IDF directly on the host. In general, Docker is recommended because of ease of use, but flashing and monitoring are [not supported](https://docs.docker.com/desktop/faqs/general/#can-i-pass-through-a-usb-device-to-a-container) on macOS since devices (USB connection) can't be attached to Docker.

### Using Docker to build ESP-IDF

Instead of installing the ESP-IDF it is also possible to build, flash, and monitor the ESP32 with docker. This has the advantage that both ROS2 and the ESP-IDF environment don't have to be installed. To build the project one just runs
```
docker run -it --rm --user espidf --volume="/etc/timezone:/etc/timezone:ro" -v  $(pwd):/micro_ros_espidf_component -v  /dev:/dev --privileged --workdir /micro_ros_espidf_component microros/esp-idf-microros:latest /bin/bash  -c "idf.py build"
```
Note that this command is just for building. To build, flash, and monitor just replace `idf.py build` with `idf.py build flash monitor`. Note that you still have to configure the WIFI Transport Layer before building.

### Install and set up ESP-IDF on Host

* Make sure you have installed all [prerequisites](https://docs.espressif.com/projects/esp-idf/en/release-v4.3/esp32/get-started/index.html#step-1-install-prerequisites).

* Download ESP-IDF 4.3 (we assume here to install it in `~/esp/` but feel free to change the location):
    ```
    mkdir -p ~/esp
    cd ~/esp
    git clone -b release/v4.3 --recursive https://github.com/espressif/esp-idf.git
    ```

* Install ESP-IDF:
    ```
    ~/esp/esp-idf/install.sh
    ```

* Set up the environment variables (note the space after the point):
    ```
    . ~/esp/esp-idf/export.sh
    ```

* The component needs `colcon` and other Python 3 packages inside the IDF virtual environment to build micro-ROS packages:
    ```
    pip3 install catkin_pkg lark-parser empy colcon-common-extensions importlib-resources
    ```

### Configuring WIFI Transport Layer

* Cd into the firmware folder and set the target
    ```
    cd firmware
    idf.py set-target esp32
    ```

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

After fixing compilation errors it may be helpful to clean and rebuild the micro-ROS library:
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
