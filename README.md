# RobotTeam package

> This is the ROS code package shared by the robot team. The team robot development code will be continuously hosted to github and will be continuously updated.
> 此为机器人小组共享 ROS 代码包，小组机器人开发代码将持续托管至github，并会保持持续更新。

## 下载代码 Download code
```shell
$ git clone hhttps://github.com/Jarrett-li-china/RobotTeam.git
```

## 串口包下载配置 Serial port package download and configuration

- 下载 download
```shell
$ git clone https://github.com/wjwwood/serial.git
```
- 配置 configuration
```shell
$ cd serial
$ mkdir build/
$ cd build/
$ cd ..
$ cmake ../ -DCMAKE_INSTALL_PREFIX=/usr/local
$ sudo make install
```

