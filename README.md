# TODO
- [ ] Add video tutorial.

# OGM2PGBM: Occupancy Grid Map to Pose Graph-based Map

Table of Contents
=================

* [Intro](#brief-intro)
* [OGM2PGBM](#Ogm2pgbm)
   * [Principle](#principle)
   * [Usage](#usage)
      * [Args](#args)
      * [Step by step](#step-by-step)
      * [Note](#note-1)
* [GMCL_CARTO](#gmcl_carto)

# Intro

This repo contains the following application:

- `OGM2PGBM`: generate pose graph-based maps on 2D occupancy grid maps (which can be created from an TLS Point cloud or a BIM/CAD model.
This pose graph-based maps can be used for accurate localization in changing and dynamic environments as demostrated in our [paper.]{https://publications.cms.bgu.tum.de/2022_ECPPM_Vega.pdf}

The following figure shows an overview of the proposed open source method.
![MethodSummary](./docs/MethodSummary.png)

- `GMCL_CARTO`: conbine the fast global localization feature of GMCL with the more accurate pose tracking performance of Cartographer

Additionally, it includes  the packages `amcl`, `gmcl`, `cartographer` and `slam_toolbox`, so that they can be used and compared with a bagfile that should be located in the mounted directory `~/workspace` easily. 


# OGM2PGBM

## Principle

The workflow of OGM2PGBM is as follows, see the function `new_map_callback(self,  grid_map)` for details:

1. **Subscribe** map from the map topic
2. **Skeletonize** the map and get its voronoi waypoint (see `self.skeletonize()`)
3. Perform a **coverage path planning** on it (see `self.CPP()`)
   - Extract the farest endpoint pair first as the start and goal point
   - Then **dilate** it with a 2x2 kernel to bold the centerline
   - Do CPP wavefront algorithm and get the sorted waypoint
4. Do a raytracing on the waypoints one by one, and publish the `/laserscan` topic (see `self.raytracer()`)

It produces `/tf`, `/clock`, `/odom`,  `/scan` topics with frame `robot_map`, `robot_odom` and `robot_base_link`.

Since `/tf` is needed, `python2.7` is used in this script. 

## Usage
### Args
This package is a standard ros package and can be launched with roslaunch command.

args:
- `map_file`: default value is `/root/workspace/map/OGM_empty.pgm.yaml`
- `record`: default value is `false`, the recorded bag can be found at `/root/.ros/ogm2pgbm_sensordata.bag`

### Step by step
0. First clone the repository.

```shell
git clone https://github.com/MigVega/Ogm2Pgbm.git
cd Ogm2Pgbm
```
With the launch file, we only need three steps to generate the base pbstream/posegraph based-map.
1. run into the docker 
```shell
bash autorun.sh
```
2. launch the application with or without the args
```shell
roslaunch ogm2pgbm ogm2pgbm.launch map_file:=/root/workspace/map/OGM_empty.pgm.yaml record:=true 
```
The target bag file will be stored under `/root/.ros/ogm2pgbm_sensordata.bag`. By default, the demo bag will also be copied into this place. So you can also skip the second step if you want.

3. After generating bagfiles, use Cartographer to generate pbstream or SLAM toolbox to generate posegraph maps.
With the following command **Cartographer** will run in offline mode, which will generate pbstream quite fast, but without any visual output in rviz.

```
roslaunch cartographer_ros ogm2pgbm_my_robot.launch bag_filename:=/root/.ros/ogm2pgbm_sensordata.bag
```
You can also launch **Slam_toolbox**. (There will be some error report in the terminal, just ignore them and wait for some seconds.)
```shell
roslaunch slam_toolbox ogm2pgbm.launch bag_filename:=/root/.ros/ogm2pgbm_sensordata.bag
```

![ogm2pgbm_posegraph](./docs/ogm2pgbm_posegraph.png)

The target pbstream file will be generated automatically at `/root/.ros/ogm2pgbm_sensordata.bag.pbstream` after . 
For slam_toolbox, you also need to click on the serialization button on the rviz plugin. The target files are also located at `/root/.ros`.

### Note
1. As it is already a package managed by catkin, if you have changed the src code or configuation inside the container, you need to do the following instructions to compile and install the package and source the env.
  ```shell
  $ catkin_make_isolated --install --use-ninja --pkg ogm2pgbm
  $ source install_isolated/setup.bash
  ```
2. All the node will be closed when the rviz is closed, including the rosbag node
3. The parameters of cartographer when using OGM2PGBM scripts to generate pbstream are as follows (see `cartographer_ros/configuration_files/ogm2pgbm_my_robot.lua`)
- remap `/scan` or `/odom` in launch file if needed


# Citation
```
@inproceedings{ vega:2022:2DLidarLocalization,
	author = {Vega, M. and Braun, A. and Borrmann, A.},
	title = {Occupancy Grid Map to Pose Graph-based Map: Robust BIM-based 2D- LiDAR Localization for Lifelong Indoor Navigation in Changing and Dynamic Environments},
	booktitle = {Proc. of European Conference on Product and Process Modeling 2022},
	year = {2022},
	month = {Sep},
	url = {https://publications.cms.bgu.tum.de/2022_ECPPM_Vega.pdf},
}
```
# GMCL_CARTO
This project combines the pros of the two algorithms, using the fast global localization feature of GMCL and the accurate pose tracking performance of Cartographer.

- First, change the bagname in the line 11 of the file `~/catkin_ws/src/gmcl_carto/gmcl_carto.py`,
- Then, make sure the bagfile is located in the directory `/root/workspace`
- At last, run the script directly `python ~/catkin_ws/src/gmcl_carto/gmcl_carto.py`

# Reference Projects
- [AMCL]{http://wiki.ros.org/amcl}
- [GMCL]{http://wiki.ros.org/gmcl}
- [SLAM_Toolbox]{https://github.com/SteveMacenski/slam_toolbox}
- [Cartographer]{https://github.com/cartographer-project/cartographer}

