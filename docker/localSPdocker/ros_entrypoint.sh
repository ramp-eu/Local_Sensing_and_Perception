#!/bin/bash
set -e

echo SP Local : OPIL version 3 is now running
# firos setup
sed -e "s/LOCALHOST/$HOST/g" -e "s/FIWAREHOST/$FIWAREHOST/g" /catkin_ws/src/firos/config/config.json.template > /catkin_ws/src/firos/config/config.json

if [ $SIMULATION == true ]
        then 
                echo "without RAN but with simulator Stage"
				echo "sleeping"
				sleep 5
				echo "continuing"
        else
				echo "sleeping"
				sleep 5
				echo "continuing"
				export ROS_MASTER_URI=$ROS_MASTER_URI
				export ROS_IP=$ROS_IP
				echo $ROS_MASTER_URI
fi

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/catkin_ws/devel/setup.sh"

FILEROBOTS=/robots.json
if test -f "$FILEROBOTS"; then
	cp robots.json /catkin_ws/src/firos/config/
fi
FILEWHITELIST=/whitelist.json
if test -f "$FILEWHITELIST"; then
	cp whitelist.json /catkin_ws/src/firos/config/
fi
FILEAMCL=/amcl_map.launch
if test -f "$FILEAMCL"; then
	cp amcl_map.launch /catkin_ws/src/localization_and_mapping/lam_simulator/launch/
fi
FILEWORLD=/map.world
if test -f "$FILEWORLD"; then
	cp map.world /catkin_ws/src/localization_and_mapping/lam_simulator/yaml/
fi
FILEROBOTSIM=/local_robot_sim.launch
if test -f "$FILEROBOTSIM"; then
	cp local_robot_sim.launch /catkin_ws/src/localization_and_mapping/sensing_and_perception/
fi
FILEROBOT=/local_robot.launch
if test -f "$FILEROBOT"; then
	cp local_robot.launch /catkin_ws/src/localization_and_mapping/sensing_and_perception/
fi
FILEPNG=/map.png
FILEYML=/map.yaml
if test -f "$FILEPNG"; then
	cp map.png /catkin_ws/src/localization_and_mapping/lam_simulator/yaml/
	if test -f "$FILEYML"; then
		cp map.yaml /catkin_ws/src/localization_and_mapping/lam_simulator/yaml/
		if [ $SIMULATION == true ]
				then 
				        echo "with simulator Stage"
				        exec roslaunch sensing_and_perception local_robot_sim.launch
				else
				        echo "without simulator"
				        exec roslaunch sensing_and_perception local_robot.launch
		fi
	fi
fi

echo "please insert a floorplan.png and floorplan.yaml file to begin!"
echo "in your docker-compose.yml put under volumes:"
echo "            - ./floorplan.yaml:/map.yaml:ro"
echo "            - ./floorplan.png:/map.png:ro"



exec  "$@"
