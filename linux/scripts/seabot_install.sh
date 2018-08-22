sudo apt install -y gpsd gpsd-clients gpsd-dbg ntp ntp-doc ntpstat socat nmap zsh git ssh cron cronutils 
sudo apt install -y screen i2c-tools htop pps-tools libgps-dev libgl1-mesa-dev
sudo apt install -y libi2c-dev libproj-dev scons

sudo apt install -y python-smbus minicom python-scipy synaptic
# Note :
# - Do not install ntpdate

chsh -s /bin/zsh
#sh -c "$(wget https://raw.githubusercontent.com/robbyrussell/oh-my-zsh/master/tools/install.sh -O -)"


# Workspace INSTALL
cd ~
git clone https://github.com/ThomasLeMezo/workspaceFlotteur.git

# ROS INSTALL
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --keyserver-options http-proxy=http://squid:3128 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt update
sudo apt install -y ros-kinetic-ros-base
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.zsh" >> ~/.zshrc
source ~/.zshrc
sudo apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential
sudo apt install -y ros-kinetic-tf ros-kinetic-angles

echo "source ~/workspaceFlotteur/devel/setup.zsh" >> ~/.zshrc

# RTIMULib2 install
git clone https://github.com/richardstechnotes/RTIMULib2.git
cd RTIMULib2/Linux
mkdir build
cd build
cmake ..
make -j4
sudo make install
sudo ldconfig

# Note NTP Install
# ./configure --enable-NMEA --enable-NMEA --enable-SHM --enable-thread-support --with-threads --enable-libopts-install

