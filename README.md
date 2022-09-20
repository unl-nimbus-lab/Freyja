# Freyja
High-level control stack for precise multirotor & ground-robot control, designed and used extensively in the Nimbus Lab.

Freyja bundles ROS2 packages that implement fast and accurate state estimators, an optimal feedback controller (LQR/LQG), and generalized interfaces for common autopilots that make it suitable for several precise and aggressive flight manuevers.
**Only in ROS2**, Freyja also includes a model-based controller for omni-directional ground robots (using mecanum-wheel drives), and a sample interface for DJI Robomasters.

The implementation uses standard control-systems style and terminology, and follows a conventional ROS architecture, thereby making it easy to substitute custom controllers, observers and communication interfaces. Freyja can remain oblivious to the specific _type_ of multirotor as long as the onboard autopilot can stabilize its attitude accurately, and can exploit differentially-flat trajectory planning methods to support feed-forward elements in the controller. Agile and precise flights can be performed _outdoors,_ even under high wind conditions, when bias compensation and RTK GPS integration is enabled.

## Overview
Freyja is a collection of **three** primary ROS packages:
- `state_manager` : interfaces several input data sources (motion capture, gps, camera estimate) to produce one unified "state_vector" [denoted $\mathbf{\vec{x}}$ in typical control schemes]. Some parts of the state vector are calculated (such as velocity), and some are merged from different callbacks. Also implements a collection of commonly used filters (Gaussian, LWMA, Median ..)
- `lqg_control`   : the core controller node that takes $\mathbf{\vec{x}}$ from `state_manager`, a reference state $\mathbf{\vec{x}_r}$, and calculates the optimal control required to regulate $\mathbf{\vec{x}}$ to $\mathbf{\vec{x}_r}$. Implements a standard LQR controller and a full state observer (Kalman) to function as a Linear Quadratic Gaussian (LQG) controller. Dynamic models for two types of vehicles are implemented: multirotors, and omni-directional ground robots.
- `autopilot_handler` : contains communication interfaces to autopilots. Currently supported autopilots are: (for flight) Pixhawk (with ArduCopter stack, px4 experimental) and Ascending Technologies; (for ground robots) DJI Robomaster.

The trajectory reference, $\mathbf{\vec{x}_r}$, can be smooth parametric curves, piecewise continuous paths, or be produced by more advanced trajectory planners.
The optional `waypoint_manager` node provides a convenience handler for providing discrete waypoints to the system, which are automatically converted to smooth trajectory references (using linear or time-optimal paths). It is possible to directly publish discrete $\mathbf{\vec{x}_r}$ references; however, the behavior then is not guaranteed to be smooth.

## Build
Clone (or submodule) the repository into the `src/` directory of your ROS workspace. Other packages in your project (such as camera processing, machine learning, trajectory generators) will sit adjacent to `Freyja/`. A recommended directory structure would look like so:
```bash
- my_awesome_project/
  | -- install/
  | -- devel/
  | -- src/
       | -- Freyja/
       | -- my_trajectory_provider/
       | -- my_image_classifier/
```
Run `colcon build` inside the project level directory (`my_awesome_project/` in the example above). Freyja will be compiled alongside your other packages, and the build products are located within the common `install/` and `devel/` directories.
> **`NO_PIXHAWK`**  <br>
The default communication interface (for ArduPilot/px4) has a dependency on `mavros` (and packages it depends on), all of which you must install ([see Wiki for how-to](https://github.com/unl-nimbus-lab/Freyja/wiki)). If you'd like to circumvent this dependency (a custom interface, different autopilot etc), you can pass `-DNO_PIXHAWK` as an argument as `colcon build --cmake-args -DNO_PIXHAWK=True`. Note that this needs you to provide some means of communicating the Freyja-generated control commands, such as `[roll,pitch,yaw,thrust]` (in respective physical SI units), to an autopilot.

> **`NO_ROBOMASTER`**  <br>
The default communication interface for ground-robots uses the `robomaster_msgs` package. This specifically targets DJI Robomaster-S1/EP vehicles and their wheel numbering. If you'd like to circumvent this dependency, you can pass `-DNO_ROBOMASTER` as an argument as `colcon build --cmake-args -DNO_ROBOMASTER=True`. Note that this needs you to provide some means of communicating the Freyja-generated wheel speeds `[omega_1,omega_2,omega_3,omega_4]` (in rads/s), to your robot.

## Run
Freyja includes a `freyja_flight.launch.yaml` file that spawns the constituent nodes, and accepts commonly used parameters as launch arguments. The simplest way to launch the full system is:
```sh
$> ros2 launch src/Freyja/freyja_flight.launch.yaml total_mass:=1.2 controller_type:=pos-vel
```
Here, `total_mass` and `controller_type` are two of the several launch arguments; others may be provided as needed.

For bigger projects that involve more packages, prepare your own launch file, and include Freyja's launch file in it.
```xml
<launch>
  <!-- launch Freyja with custom arguments -->
  <include file="src/Freyja/freyja_flight.launch.yaml" >
    <arg name="total_mass"                value="1.2" />
    <arg name="state_source"              value="tf_mocap" />
    <arg name="thrust_scaler"             value="100" />
    <arg name="use_waypoint_handler"      value="true" />
    <!-- see launch file for more available arguments -->
  </include>
  
  <node name="my_traj" pkg="my_trajectory_provider" type="my_glamorous_trajectory" />
  :
  :
</launch>  
```
In the example above, the launch argument `use_waypoint_handler` is set to `true` for providing discrete waypoints (and not reference states). See the launch file for more available arguments.

## Wiki
Check out [Freyja's wiki](https://github.com/unl-nimbus-lab/Freyja/wiki) to get more details on everything, from a basic multirotor build, to advanced configuration and set up, all the way to how trajectories can be written.

## License, Credits and Usage
Freyja was developed originally in the [Nimbus Lab](https://nimbus.unl.edu), with rigorous testing under a multitude of operational scenarios and is used internally for various projects. Generous thanks to members who have tested and reported bugs, issues and feature limitations. The software is public under the GNU GPLv3 license. Please use wisely, and recommend improvements!

If you use Freyja for your work, you can site the ICRA 2021 paper that describes it in full detail:

```BibTeX
inproceedings{shankar2021freyja,
  title={Freyja: A full multirotor system for agile \& precise outdoor flights},
  author={Shankar, Ajay and Elbaum, Sebastian and Detweiler, Carrick},
  booktitle={2021 IEEE International Conference on Robotics and Automation (ICRA)},
  pages={217--223},
  year={2021},
  organization={IEEE}
}
```
>```bash
>"Freyja: A Full Multirotor System for Agile & Precise Outdoor Flights",
>A. Shankar, S. Elbaum, C. Detweiler;
>IEEE International Conference on Robotics and Automation (ICRA), 2021.
