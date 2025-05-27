## Navegação:
* Navegação via WASD;
* Ativação dos motores quando o nodo de localização está ativo;
* Loiter (em testes).

### Passos:
```
cd ~/vrx_ws
colcon build --merge-install
source /opt/ros/humble/setup.bash
. install/setup.bash
```

Para executar o **timed_thrust_node**:
```
ros2 run vrx_navigation timed_thrust_node
```