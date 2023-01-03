
# Librealsense installation procedure

Update Ubuntu distribution, including getting the latest stable kernel

```bash
sudo apt-get update && sudo apt-get upgrade && sudo apt-get dist-upgrade
```
Then install the following packages

```bash
apt-get install python3-sphinx build-essentials git libssl-dev libusb-1.0-0-dev pkg-config libgtk-3-dev
```

Clone the librealsense:

```bash
git clone https://github.com/IntelRealSense/librealsense/tree/R250
```

and compile as follows:

```bash
mkdir build 
cd build 
cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_PYTHON_BINDINGS=ON ..
make -j<x>
sudo make install
```
