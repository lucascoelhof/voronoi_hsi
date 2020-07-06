# Voronoi HSI (Human-Swarm Interaction)

Multi **robot coverage control** in non-convex environments using **ROS**.

![Voronoi HSI Example](assets/voronoi_hsi-example.jpg)

This is a ROS implementation of a multi-robot coverage control strategy based in for the paper [Voronoi Multi-Robot Coverage Control in Non-Convex Environments With Human Interaction in Virtual Reality](https://www.researchgate.net/publication/329624395_VORONOI_MULTI-ROBOT_COVERAGE_CONTROL_IN_NON-CONVEX_ENVIRONMENTS_WITH_HUMAN_INTERACTION_IN_VIRTUAL_REALITY) to address coverage in non-convex environments and adapt to performance variations.

It also has an add-on software for Unity that allows humans to interact with the robotic swarm, influencing their coverage task: [Voronoi Unity Teleoperation](https://github.com/lucascoelhof/VoronoiUnityTeleoperation)

## Getting started

Dependencies:

* [Stage](http://wiki.ros.org/stage)
* [matplotlib](https://matplotlib.org/)
* [ar_track_alvar](http://wiki.ros.org/ar_track_alvar)
* [gctronic/epuck_driver](https://github.com/gctronic/epuck_driver)

Download the code, place it on your catkin environment, compile it using ``catkin_make`` and run using:

```
roslaunch voronoi_hsi voronoi_8_with_app.launch
```

If you want to use real robots (epucks) instead of simulating it on Stage, you will need the driver [gctronic/epuck_driver](https://github.com/gctronic/epuck_driver). Run it using:

```
roslaunch voronoi_hsi voronoi_8_with_app.launch real_robot:=true
```

## Human-Swarm Interaction Integration

This application can run standalone, but it can also run integrated with a VR application, which allows users to interact with the
robotic swarm in the coverage task.

Please follow instructions at [Voronoi Unity Teleoperation](https://github.com/lucascoelhof/VoronoiUnityTeleoperation) to setup and
run the VR application.

## License

This project is distibuted under the [BSD 3-Clause License](LICENSE).

## Acknowlegement

If you found this code useful for your scientific research, please remind to cite the paper that originated this code implementation:
```
@inproceedings{inproceedings,
author = {Figueiredo, Lucas and Lelis de Carvalho, Italo and Pimenta, Luciano},
year = {2018},
month = {01},
pages = {},
title = {VORONOI MULTI-ROBOT COVERAGE CONTROL IN NON-CONVEX ENVIRONMENTS WITH HUMAN INTERACTION IN VIRTUAL REALITY},
doi = {10.20906/CPS/CBA2018-0563}
}
```

