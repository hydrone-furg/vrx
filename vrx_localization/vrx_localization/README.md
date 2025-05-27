## Localização:
* GPS (/wamv/sensors/gps/gps/fix);
* IMU (/wamv/sensors/imu/imu/data).

### Passos:
```
cd ~/vrx_ws
colcon build --merge-install
source /opt/ros/humble/setup.bash
. install/setup.bash
```

Para executar o **localization_node**:
```
ros2 run vrx_localization localization_node
```

Para visualizar o tópico criado **/wamv/geopose**:
```
ros2 topic echo /wamv/geopose
```