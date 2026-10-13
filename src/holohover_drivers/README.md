# Holohover Drivers
This package contains the code for the Radxa SBC.

The components of the Hovercraft are the following:
- Radxa Zero 3W.
- Flywoo GOKU HEX GN405 Nano flight controller.
- PMW3389DM mouse sensor.

The Radxa is connected via a UART serial to the Betaflight and via SPI to the mouse sensor.

In the `kernel_modules` directory there is a custom SPI module that has to be compiled in order to communicate with the mouse sensor.

## Nodes
- `holohover_fc` - the node that connects to the flight controller. It subscribes to the control topic and publishes the data from the IMU and the battery voltage.
- `holohover_mouse_sensor` - reads data from Mouse sensor and publishes it.

The topic names are set in the code without a starting '/', so that if the nodes are started in a namespace, the topics are related to that same namespace.

## Setup of a new hovercraft
### Configuring Betaflight

In Ports enable `UART2` and set the MSP configuration to `1000000`. Save and reboot.

In CLI run the following commands:
```
set serial_update_rate_hz = 2000
set serial_delay_ms = 20000
save
```
Double check that
```
get serial_update_rate_hz
```
returns `serial_update_rate_hz = 2000`.

In Motors change the mixer to `HEX X` and the ESC/Motor protocol to `DSHOT600` and `Save and Reboot`.

Next, we have to reorder the motors. In the motor section, click `Reorder motors` and reorder the motor in the following order: A1, A2, B1, B2, C1, C2.

### Configuring Radxa Zero 3W

Download the [ubuntu-22.04-preinstalled-server-arm64-radxa-zero3.img.xz](https://joshua-riek.github.io/ubuntu-rockchip-download/boards/radxa-zero3.html) image and flash it onto a SD card using something like [balenaEtcher](https://etcher.balena.io/).

Mound the just flashed SD card (works on Linux but not on macOS?) and configure your wifi (network-config) accordingly along the lines of
```
network:
  version: 2
  ethernets:
    zz-all-en:
      match:
        name: "en*"
      dhcp4: true
      optional: true
    zz-all-eth:
      match:
        name: "eth*"
      dhcp4: true
      optional: true

  wifis:
    wlan0:
      dhcp4: true
      optional: true
      access-points:
        "TP-Link_08B0_5G":
          password: "78122987"
```
Insert the SD card into the zero 3 and let it boot. After some time (give it a couple minutes) you should be able to see a new device on the network. Connect to it using
```bash
ssh ubuntu@192.168.0.xxx
```
with password `ubuntu`. You will be prompted to change it, so do so.

We need to install some additional libraries and docker, so run the following commands:
```bash
sudo apt update
sudo apt install gpiod ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

sudo apt-get install docker-ce docker-ce-cli containerd.io docker-compose-plugin

sudo groupadd docker
sudo usermod -aG docker $USER
```

Add the following to `/etc/docker/daemon.json`
```
{
    "insecure-registries" : [ "192.168.0.70:5000" ]
}
```

We are going to use a custom spi driver. To build and install the driver cd into the `holohover/holohover_drivers/kernel_modules` folder and run the following commands:
```bash
make
sudo cp spideve.ko /lib/modules/$(uname -r)/kernel/drivers/spi/spideve.ko
sudo depmod
```
Edit `/etc/modules` and add a new line with `spideve`.

To get the connected uart and spi peripherals working we need to modify the device tree. Go into the folder `/usr/lib/firmware/$(uname -r)/device-tree/rockchip/overlay` and create a new file `radxa-zero3-uart2-m0-spi3-m1-cs0.dts` with following content:
```
/dts-v1/;
/plugin/;

/ {
	metadata {
		title = "Enable UART2 M0 and SPI3 M1 CS0";
		compatible = "radxa,zero3";
		category = "misc";
		description = "Enable UART2 M0 and SPI3 M1 CS0 on Zero 3W.";
	};

	fragment@0 {
		target = <&uart2>;

		__overlay__ {
			status = "okay";
			pinctrl-0 = <&uart2m0_xfer>;
		};
	};

    fragment@1 {
		target = <&fiq_debugger>;

		__overlay__ {
			status = "disabled";
		};
	};

    fragment@2 {
		target = <&spi3>;

		__overlay__ {
			status = "okay";
			#address-cells = <1>;
			#size-cells = <0>;
			num-cs = <1>; pinctrl-names = "default", "high_speed";
			pinctrl-0 = <&spi3m1_cs0 &spi3m1_pins>;
			pinctrl-1 = <&spi3m1_cs0 &spi3m1_pins_hs>;
			max-freq = <2000000>;

			spideve@0 {
				compatible = "rockchip,spideve";
				status = "okay";
				reg = <0>;
			};
		};
	};
};
```
Run the following command to compile the dts
```bash
sudo dtc -@ -O dtb -o radxa-zero3-uart2-m0-spi3-m1-cs0.dtbo radxa-zero3-uart2-m0-spi3-m1-cs0.dts
```
Edit the file `/etc/default/u-boot` and modify the variable `U_BOOT_FDT_OVERLAYS` as
```
U_BOOT_FDT_OVERLAYS="device-tree/rockchip/overlay/radxa-zero3-uart2-m0-spi3-m1-cs0.dtbo"
```
To not have have the system console printed out on the uart2 port, modify the content of `/etc/kernel/cmdline` to the following:
```
rootwait rw console=tty1 cgroup_enable=cpuset cgroup_memory=1 cgroup_enable=memory
```
After saving the config files, use the `u-boot-update` utility to apply it:
```bash
sudo u-boot-update
```
Reboot the system with
```bash
sudo reboot
```
You should now see two new devices: `/dev/ttyS2` and `/dev/spideve3.0`

### Checking and Reversing Motor Directions

To reverse the motor directions we have to flash the firmware of the ESCs. For this we use [BLHeli](https://github.com/blheli-configurator/blheli-configurator/releases).

Connect to the flight controller and connect the battery (ESCs need to be powered). Open BLHeli and `Read Setup`. You should now see a list of 6 ESCs. To change the direction, change the motor direction from `Normal` to `Reversed`, click `FLASH FIRMWARE` and flash it using the same ESC type and version.


# ToDo
add chrony installation, clone of the repo, .. - all of this was already added in an instruction file, but I cannot find it
