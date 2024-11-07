# Comandos para executar o programa

## Dentro da pasta ardupilot/Rover para iniciar simulação SITL do Ardupilot e a ground station MAVProxy para controlar a simulação:
Esse comando deve ser o primeiro. Se necessário, atualizar a localização SEParnaiba para ARGO 
para ficar de acordo com o mundo do Thiago
```bash
./gzrover.sh
```
Usar o abaixo, pois o de cima está dando conflito com o arquivo .param
```bash
sim_vehicle.py -v Rover -f gazebo-rover -L ARGO --console --map
```
```bash
sim_vehicle.py -v Rover -f gazebo-rover -L ARGO nrover.param --console --map
sim_vehicle.py -v Rover -f gazebo-rover -L ARGO --console --map --add-param-file nrover.param
sim_vehicle.py -v Rover -f gazebo-rover -L ARGO nrover.param --console --map --out=udp:127.0.0.1:14550 -I 0
sim_vehicle.py -v Rover -f gazebo-rover -L ARGO --console --map --out=udp:127.0.0.1:14550 -I 0
```
## Dentro da pasta ardupilot_gazebo/launch:
Este deve ser o segundo comando
```bash
roslaunch navigation.launch
roslaunch teste_2rovers.launch 
roslaunch teste_2rovers.launch fcu_url:=udp://:14550@14555 fcu_url_2:=udp://:14570@14575 fcu_url_3:=udp://:14590@14595
```
## Dentro da pasta deste programa:
```bash
pyhton3 programa.py
```
## Dentro da pasta em que foi extraído o arquivo .zip do Mission Planner:
```bash
mono MissionPlanner.exe
```
## Lembrete:
O robô do Thiago não vem naturalmente com o Lidar, é preciso acrescentá-lo dentro
da pasta, no meu computador, catkin_ws/src/rover-argo-gazebo/rover-argo-gazebo/models/rover_argo_NZero