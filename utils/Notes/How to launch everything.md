cd /workspace/SIM/Worlds && gz sim -v4 -r world_dummies.sdf

sim_vehicle.py -v ArduCopter -f gazebo-iris   --model JSON   -w   --add-param-file=/workspace/test.parm   --out=udpout:127.0.0.1:15000   --out=udpout:127.0.0.1:15001   --console --out=udpout:127.0.0.1:15003 --out=udpout:127.0.0.1:14551

ros2 



