# Voronoi HSI (Human-Swarm Interaction)

Multi robot coverage control in non-convex environments using ROS.

This is a ROS implementation of a multi-robot coverage control strategy based in [1] and [2] to address coverage in non-convex
environments and adapt to performance variations. It also has a plugin software that allows humans to interact with the robotic
swarm, influencing their coverage task.

# Getting started
Dependencies:
* Stage
* matplotlib

Download the code, compile it and run using:
```
roslaunch voronoi_hsi voronoi_8_with_app.launch
```

We've also developed experiment using the e-puck robot and a driver available at [gctronic/epuck_driver_cpp
](https://github.com/gctronic/epuck_driver_cpp).


# Human-Swarm Interaction integration

This application can run standalone, but it can also run integrated with a VR application, which allows users to interact with the
robotic swarm in the coverage task.

Please follow instructions at [Voronoi Unity Teleoperation](https://github.com/lucascoelhof/VoronoiUnityTeleoperation) to setup and
run the VR application.


# References
1. Bhattacharya, S., Ghrist, R. and Kumar, V., 2014. Multi-robot coverage and exploration on Riemannian manifolds with boundaries. The International Journal of Robotics Research, 33(1), pp.113-137.
2. Pierson, A., Figueiredo, L.C., Pimenta, L.C. and Schwager, M., 2015, May. Adapting to performance variations in multi-robot coverage. In Robotics and Automation (ICRA), 2015 IEEE International Conference on (pp. 415-420). IEEE.
