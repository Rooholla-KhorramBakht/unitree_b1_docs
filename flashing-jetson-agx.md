# Jetson Xavier AGX

## Flashing New OS (Internal Flash)
To flash the board with the required OS, first download the driver package bsp and sample rootfs from [here](https://developer.nvidia.com/embedded/jetson-linux-archive). Assuming `~/devel` as the working directory, we run the following commands to prepare the image for flashing ([source](https://docs.nvidia.com/jetson/archives/r34.1/DeveloperGuide/text/IN/QuickStart.html#in-quickstart)):

```bash
cd ~/devel
tar xf <driber package bsp file path>
cd Linux_for_Tegra/rootfs/
sudo tar xpf <sample rootfs file path>
cd ..
sudo ./apply_binaries.sh
```

Then, put the board into recovery mode and run the following command:

'''bash
sudo ./flash.sh ${BOARD} mmcblk0p1
'''

The `BOARD` environmental variable holds the name of the development board we use. For Xavier AGX this value is `jetson-agx-xavier-devkit` and for the Orin it is `jetson-agx-orin-devkit`. 

After the flashing process is completed, the board reboots where the user needs to setup the account and passwords to finalize the installation. 

## Backup and Recovery (Internal Flash)
### Creating a Backup
Imagine we have installed bunch of packages and want to create a backup of the operating system for later recovery or cloning into new new boards. Doing so using the flash utility is simple and may be carried out as follows (as explaned [here](https://docs.nvidia.com/jetson/archives/l4t-archived/l4t-3261/index.html#page/Tegra%20Linux%20Driver%20Package%20Development%20Guide/flashing.html#wwpID0E06P0HA)):

First put the board into recovery mode. Then, run the following command to start the backup process:

```bash
sudo ./flash.sh -r -k APP -G <clone> <board> mmcblk0p1
```
where:

- `<clone>` determines the names of the copies.
- `<board>` specifies the configuration of the target device.

This step creates two copies of `<clone>` in the `<top>` directory: a sparsed image (smaller than the original) named '<clone>', and an exact copy named `<clone>.raw`.
### Recovering the Backup

The generated backup from the previous section may be flashed into a new board as follows. First, copy the generated backup file from the previous section into the bootloader directory of the BSP folder:

```bash
sudo cp <clone>.img <bsp path>/bootloader/system.img
```

Then, put the board into the recovery mode and run the following command:

- If the target board has already been flashed, reflash the clone image to the APP partition. Enter this command:
```bash
sudo ./flash.sh -r -k APP <board> mmcblk0p1
```
- If the target board has never been flashed, flash all of the board’s partitions. Enter this command:
```bash
sudo ./flash.sh -r <board> mmcblk0p1
```