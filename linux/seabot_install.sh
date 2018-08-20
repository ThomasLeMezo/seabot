sudo apt install -y gpsd gpsd-clients gpsd-dbg ntp ntp-doc ntpstat socat nmap zsh git ssh cron cronutils screen i2c-tools htop pps-tools

# Note :
# - Do not install ntpdate

chsh -s /bin/zsh
#sh -c "$(wget https://raw.githubusercontent.com/robbyrussell/oh-my-zsh/master/tools/install.sh -O -)"


# Workspace INSTALL
cd ~
git clone https://github.com/ThomasLeMezo/workspaceFlotteur.git

# ROS INSTALL
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-ros-base
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.zsh" >> ~/.zshrc
source ~/.zshrc
sudo apt-get install python-rosinstall python-rosinstall-generator python-wstool build-essential

echo "source ~/workspaceFlotteur/devel/setup.zsh" >> ~/.zshrc