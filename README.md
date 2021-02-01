# mod.iot.sp.local

OPIL module: Local Sensing & Perception (Local SP)
Innovation Centre Nikola Tesla, written by Marija from May 6th 2018, contintributed by Goran, Jura, Petki and Ana

# Creating the docker image

This is the local SP. The docker image is created by executing in the root folder of this package:

```
docker build -t "localsp:test" -f docker/localSPdocker/Dockerfile .
```

Test built docker container by starting `docker-compose up` from the test folder. Make sure you started the OCB before. You can start it from the Central SP by starting `docker-compose up` from the folder `test/docker_compose_files/Central_SP_docker.

# Quick start from source

Documentation about the Local SP module can be found here <https://opil-documentation.readthedocs.io/en/latest/SP/Local_SP_Getting_Started.html>.

Type `catkin_init_workspace` in the src folder of this repository. Then, compile it with `catkin_make` in one folder up.
```
cd ..
catkin_make
```
Put the `setup.bash` script from the newly created `devel` folder to your `.bashrc` file:

```
source (your path to the repo folder)/mod.iot.sp.local/devel/setup.bash
```

Open the new terminal tab and command `roscd` should navigate to the repo folder.

First, start the Stage simulator with the loaded example map and one AGV inside the map (red) and one box as the unknown obstacle (green) and AMCL localization:

```
terminal 1: roslaunch lam_simulator AndaOmnidriveamcltestZagrebdemo.launch
```
Then, launch the module for creating the topic for Pose with Covariance of the AGV:

```
terminal 2: roslaunch sensing_and_perception send_posewithcovariance.launch 
```
Finally, launch the calculation of local map updates that the AGV sees as new obstacles which are not mapped in the initial map:
```
terminal 3: roslaunch mapupdates startmapupdates.launch
```

