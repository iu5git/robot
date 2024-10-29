# Робот-гид

ветка robot - Ros на роботе

ветка robot-user - Ros на сервере 

## Cсылки
###### Репозиторий
	https://github.com/kravtandr/bbot/tree/robot-user

###### Методичка по ROS
	https://github.com/kravtandr/ros_metod/blob/master/Методичка%20ROS.md

###### Подробное описание проекта от Амперки
	https://amperka.ru/blogs/projects/abot-robot-part-1

###### Для настройки локальной сети
	https://husarnet.com/

![Image alt](https://github.com/kravtandr/ros_metod/raw/master/images/robot.png)

Для начала работы потребуется Ubuntu Focal (20.04) . Именно Ubuntu Focal (20.04), другие версии не подойдут. 

*Предупреждение: Если у вас apple silicon или видеокарта AMD, то gazebo, rviz и rqt могут работать некорректно.*
## Основные команды
**roscore** - запуск ядра ROS

**rostopic list** - просмотр всех запущенных топиков

**source devel/setup.bash** - инициализация констант и переменных среды для работы с ROS в конкретной сущности терминала 

**rosrun** - позволяет запускать исполняемый файл (ноду) в произвольном пакете из любого места без указания его имени.

**roslaunch** - позволяет запускать .launch файлы, в которых может быть много различных нод.

**catkin_make** - билд проекта

**rqt** - запуск утилиты rqt
## Buld проекта
##### Для сборки используется catkin
##### Устанавливаем catkin
	sudo apt-get install -y python3-catkin-tools
##### Устанавливаем зависимости
  	sudo apt-get install -y python3-rosdep python3-rosinstall python3-rosinstall-generator \python3-wstool build-essential ros-noetic-joint-state-publisher-gui net-tools \ros-noetic-pid ros-noetic-joy xboxdrv ros-noetic-gmapping ros-noetic-joy \ros-noetic-map-server ros-noetic-navigation
##### Переходим в папку проекта и экспортируем переменные среды с помощью скрипта
   	cd robot && source devel/setup.sh
##### Билдим проект
	catkin_make
Если не сбилдилось удалить старую папку build и повторить catkin_make 
## Запуск робота
	cd ros && source devel/setup.bash &&  roslaunch bbot_driver abot_drivers.launch 

## Запуск сервера
	- Запуск ядра
	roscore
	
	- Запуск бекенда на джанго
	cd ~/Desktop/ROS_ON_SERVER/robo/ServerPy	
	sudo python3 manage.py runserver 127.0.0.1:8080
	
	- Запуск фронтенда на реакт
	cd ~/Desktop/ROS_ON_SERVER/robo/WebClient	
	cd
	
	- Запуск листнера вебсоткета для передачи в топики ROS
	cd ~/ros/Scripts
	python3 WSListner_RosPublisher.py

	- Запуск визуализации
	cd ros && source devel/setup.bash && roslaunch abot_description display_movement.launch
	- Запуск slam
	roslaunch abot_description bringup.launch



## Запуск ручного управления
###### Server 
	sudo xboxdrv --silent
	rosparam set joy_node/dev "/dev/input/js1"
	rosrun joy joy_node
	roslaunch abot_description bringup.launch  

## Сохранить карту
	cd ~/ros/src/abot_slam/maps/  
	rosrun map_server map_saver -f map1  


## Настройка сети ROS
![Image alt](https://github.com/kravtandr/ros_metod/raw/master/images/network.png)

 Документация https://wiki.ros.org/ROS/Tutorials/MultipleMachines
### При подключении к новой сети

1. sudo nano /etc/hosts
	прописать нужные айпи

2. Sudo nano ~/.bashrc

	 Robot-user
		echo "ROS_MASTER_URI=http://robot-user:11311" >> ~/.bashrc
		echo "ROS_HOSTNAME=robot-user" >> ~/.bashrc
		echo "ROS_IP=192.XX.XX.XX" >> ~/.bashrc
		
	Robot 
		echo "ROS_MASTER_URI=http://robot-user:11311" >> ~/.bashrc
		echo "ROS_HOSTNAME=robot" >> ~/.bashrc
		echo "ROS_IP=192.XX.XX.XX" >> ~/.bashrc

3. Перезагружаем настройки командной строки
	source ~/.bashrc


### Конфиг wpa_suplicant
	/etc/wpa_supplicant/wpa_supplicant.conf
### Переключение на другую сеть
	sudo wpa_cli -i wlan0 list_networks
	sudo wpa_cli -i wlan0 select_network 0

### Диаграмма развертывания 
![Image alt](https://github.com/kravtandr/ros_metod/raw/master/images/deployment.png)

### Структурная схема робота 
![Image alt](https://github.com/kravtandr/ros_metod/raw/master/images/hard.png)

### FAQ 
###### Описание: Ошибка в `rosrun tf view_frames`
###### Решение: менять файл через консоль /opt/ros/noetic/lib/tf
	https://github.com/ros/geometry/pull/193/files#diff-6752a402ecb9165369200c2f88abbe0efa471d0b69f2c9e6d907b71f0189cdba


###### Описание: **CMake Error**  controller_manager-config.cmake:
	**CMake Error** at /opt/ros/noetic/share/catkin/cmake/catkinConfig.cmake:83 (find_package):
	  Could not find a package configuration file provided by
	  "controller_manager" with any of the following names:
	    controller_managerConfig.cmake
	    controller_manager-config.cmake
######  Решение: 
	sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
	sudo apt update
	sudo apt-get install ros-noetic-controller-manager
	sudo apt-get install ros-noetic-ros-control ros-noetic-ros-controllers

###### Описание: подключение геймпада
###### Решение:
	https://howchoo.com/pi/xbox-controller-raspberry-pi
