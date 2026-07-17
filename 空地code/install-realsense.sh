sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp >/dev/null

sudo apt-get install apt-transport-https -y

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" |
    sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update

# sudo apt install librealsense2=2.53.1-0~realsense0.8250 librealsense2-dkms=1.3.18-0ubuntu1 librealsense2-utils=2.53.1-0~realsense0.8250 librealsense2-gl=2.53.1-0~realsense0.8250
sudo apt install librealsense2=2.51.1-0~realsense0.7526 librealsense2-dkms=1.3.18-0ubuntu1 librealsense2-utils=2.51.1-0~realsense0.7526 librealsense2-gl=2.51.1-0~realsense0.7526 librealsense2-net=2.51.1-0~realsense0.7526

# sudo apt install librealsense2-dev=2.53.1-0~realsense0.8250 librealsense2-dbg=2.53.1-0~realsense0.8250
sudo apt install librealsense2-dev=2.51.1-0~realsense0.7526 librealsense2-dbg=2.51.1-0~realsense0.7526

dpkg -l | grep librealsense2 | awk '{print $2}' | xargs sudo apt-mark hold
