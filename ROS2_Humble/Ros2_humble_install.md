
# 1. 设置utf8语言环境

```shell

```

```shell
locale  

sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings
```


# 2. 设置源

```shell
sudo apt install software-properties-common
sudo add-apt-repository universe
```

使用 apt 添加 ROS 2 GPG 密钥

```shell
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

接下来将软件源添加到您的源列表中

```shell
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

# 3. 安装ros2

```shell
sudo apt-get update
sudo apt-get upgrade
```

桌面安装（推荐）：ROS、RViz、演示、教程。

```shell
sudo apt install ros-humble-desktop
```

添加环境变量

```shell
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
也就是在~/.bashrc文件后面添加`source /opt/ros/humble/setup.bash`

# 安装 colcon

```shell
sudo apt install python3-colcon-common-extensions
```

# 验证

```shell
wub@wub:~$ ros2 run demo_nodes_cpp talker
[INFO] [1714036035.679785136] [talker]: Publishing: 'Hello World: 1'
[INFO] [1714036036.679330436] [talker]: Publishing: 'Hello World: 2'
[INFO] [1714036037.679588162] [talker]: Publishing: 'Hello World: 3'
[INFO] [1714036038.679293145] [talker]: Publishing: 'Hello World: 4'

# 另外建一个终端
wub@wub:~$ ros2 run demo_nodes_py listener
[INFO] [1714036273.699201424] [listener]: I heard: [Hello World: 239]
[INFO] [1714036274.671316847] [listener]: I heard: [Hello World: 240]
[INFO] [1714036275.670648235] [listener]: I heard: [Hello World: 241]

```


# gazebo

## 安装


```bash
sudo apt install ros-humble-gazebo-*
ros2 launch gazebo_ros gazebo.launch.py
```


# rqt

## 安装

```bash
sudo apt install ros-humble-rqt
```

# tf2

```bash
sudo apt install ros-humble-tf2-tools
```






