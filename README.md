# :sauropod: nanosaur_robot

[![Docker Builder CI](https://github.com/rnanosaur/nanosaur_robot/actions/workflows/docker-image.yml/badge.svg)](https://github.com/rnanosaur/nanosaur_robot/actions/workflows/docker-image.yml)

NanoSaur is a little tracked robot ROS2 enabled, made for an NVIDIA Jetson Nano

* Website: [nanosaur.ai](https://nanosaur.ai)
* Do you need an help? [Discord](https://discord.gg/NSrC52P5mw)
* For technical details follow [wiki](https://github.com/rnanosaur/nanosaur/wiki)
* Something wrong? Open an [issue](https://github.com/rnanosaur/nanosaur/issues)

## Develop nanosaur system
```
git clone https://github.com/rnanosaur/nanosaur.git
sudo bash nanosaur/nanosaur_bringup/scripts/install.sh
```

## Run docker container

https://answers.ros.org/question/358453/ros2-docker-multiple-hosts/

```
docker run --runtime nvidia -it --rm  --network host --device /dev/i2c-1 -v /tmp/argus_socket:/tmp/argus_socket -v $HOME/nanosaur:/opt/ros_ws/src/nanosaur nanosaur/nanosaur:latest bash
```

## Detect I2C devices

Install I2C tools and detect all devices

```
sudo apt-get install -y python-smbus
sudo apt-get install -y i2c-tools
```

```
sudo i2cdetect -y -r 1
```

Devices:
* **3C** left Display
* **3D** right display
* **60** motor driver

## Camera fail

If the camera fail do:

```
sudo systemctl restart nvargus-daemon
```

And rerun the docker container