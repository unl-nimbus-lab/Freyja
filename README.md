# Freyja
High-level flight stack for precise multirotor control, designed and used extensively in the Nimbus Lab.

Freyja bundles ROS packages that implement fast and accurate state estimators, an optimal feedback controller (LQR/LQG), and generalized interfaces for common autopilots that make it suitable for several precise and aggressive flight manuevers. The implementation uses standard control-systems style and terminology, and follows a conventional ROS architecture, thereby making it easy to substitute custom controllers, observers and communication interfaces. Freyja can remain oblivious to the specific _type_ of multirotor as long as the onboard autopilot can stabilize its attitude accurately, and can exploit differentially-flat trajectory planning methods to support feed-forward elements in the controller. Agile and precise flights can be performed _outdoors,_ even under high wind conditions, when bias compensation and RTK GPS integration is enabled.

## Overview
Freyja is a collection of four primary ROS packages:
- `state_manager` : interfaces several input data sources (motion capture, gps, camera estimate) to produce one `state_vector` [denoted **x** in typical control schemes]. Some parts of state_vector are calculated (such as velocity), and some are merged from different callbacks. Also implements a collection of commonly used filters (Gaussian, LWMA, Median ..)
- `lqg_control`   : the core controller node that takes **x** from `state_manager`, a reference state **x<sub>r</sub>**, and calculates the optimal control required to regulate **x** to **x<sub>r</sub>**. Implements a standard LQR controller and a full state observer (Kalman) to function as a Linear Quadratic Gaussian (LQG) controller.
- `autopilot_handler` : contains communication interfaces to autopilots. Currently supported autopilots are Pixhawk (with ArduCopter stack, px4 experimental) and Ascending Technologies.
- `freyja_trajectory_provider`  : an example trajectory provider node that produces **x<sub>r</sub>**. In common use, this node is typically suppressed (using a launch argument), and users will write their own custom trajectory node that represents their particular flight objectives. It is possible for **x<sub>r</sub>** to be simple discrete waypoints, piecewise continuous paths, or the product of an advanced trajectory planner that incorporates spatial and temporal shapes of trajectories. Path derivatives up to the 2nd order are supported.

## Build
Clone the repository into the `src/` directory of your ROS workspace. Other packages in your project (such as camera processing, machine learning, trajectory generators) will sit adjacent to `Freyja/`. A recommended directory structure would look like so:
```bash
- my_awesome_flight_project/
  | -- build/
  | -- devel/
  | -- src/
       | -- Freyja/
       | -- my_trajectory_provider/
       | -- my_image_classifier/
```
Run `catkin_make` or `catkin build` inside the project level directory (`my_awesome_flight_project/` in the example above). Freyja will be compiled alongside your other packages, and the build products are located within the common `build/` and `devel/` directories.

## Run
Freyja includes a `freyja_controller.launch` file that spawns the constituent nodes, and accepts commonly used parameters as launch arguments. The simplest way to launch the full system is:
```sh
$> roslaunch src/Freyja/freyja_controller.launch total_mass:=1.2 start_rosbag:=true
```
Here, `total_mass` and `start_rosbag` are two of the launch arguments; others may be provided as needed. When flying indoors, `vicon_topic` might be a commonly used and required argument.

For bigger projects that involve more packages, prepare your own launch file, and include Freyja's launch file in it.
```xml
<launch>
  <!-- launch Freyja with custom arguments -->
  <include file="src/Freyja/freyja_controller.launch" >
    <arg name="total_mass"                value="1.2" />
    <arg name="vicon_topic"               value="/vicon/FLH1/FLH1" />
    <arg name="start_rosbag"              value="false" />
    <arg name="use_external_trajectory"   value="true" />
  </include>
  
  <node name="my_traj" pkg="my_trajectory_provider" type="my_glamorous_trajectory" />
  :
  :
</launch>  
```
In the example above, the launch argument `use_external_trajectory` is set to true since you are expected to provide your own custom trajectory source (and not use Freyja's built-in examples).

## Wiki
Check out [Freyja's wiki](https://github.com/unl-nimbus-lab/Freyja/wiki) to get more details on everything, from a basic multirotor build, to advanced configuration and set up, all the way to how trajectories can be written.

## License, Credits and Usage
Freyja is developed in the [Nimbus Lab](https://nimbus.unl.edu), with rigorous testing under a multitude of operational scenarios and is used internally for various projects. Generous thanks to members who have tested and reported bugs, issues and feature limitations. The software is public under the GNU GPLv3 license. Please use wisely, and recommend improvements!
