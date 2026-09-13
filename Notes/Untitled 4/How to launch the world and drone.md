## 1. Clone the repo



## 2.  Build docker

```
xhost +local:root
docker compose build
docker compose up -d
```

## 3. Building ardupilot and ardupilot gazebo

## Installing ros2 humble + slam_toolbox 


## 4. Installing Prerequisites for ardupilot 
```
cd /workspace/ardupilot
Tools/environment_install/install-prereqs-ubuntu.sh -y
```
#### 5. Set up

```
export PATH=/workspace/ardupilot/Tools/autotest:$PATH
export PATH=$HOME/.local/bin:$PATH
echo 'export PATH=$HOME/.local/bin:$PATH' >> ~/.bashrc
```

#### 6. Launching a docker terminal
```
docker exec -it nidar-container bash
```
### 7. Launching the world 

In a docker terminal
```
cd /workspace/SIM/Worlds && gz sim -v4 -r combined_arena.sdf

```

### 8. Launching the drone 
In another docker terminal
```
cd /workspace/ardupilot/ArduCopter
	`sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON -w --add-param-file=/workspace/no_gps.parm --console`
```


