# Unitree Go1 Gait Generation

A ROS Noetic workspace for running Unitree Go1 locomotion in Gazebo Classic, built on Unitree's `unitree_guide`. Use it to stand the simulated robot, drive it with the keyboard, and explore trot, experimental three-beat canter, and gallop.

**The canter and gallop controllers are simulation-only.** Gazebo results are not hardware validation. Do not deploy them on a physical Go1 without a separate hardware-validation and rig-testing process.

## Available gaits

| Gait | How to use it | Status |
|---|---|---|
| Trot | Fixed stand → `4` | Default locomotion mode |
| Walk/crawl | Select the commented crawl configuration in `main.cpp`, rebuild, then use `4` | Simulation configuration; no separate keyboard mode |
| Canter | Fixed stand → `6`, then request movement | Experimental simulation controller |
| Gallop | Fixed stand → `7`, then request movement | Experimental simulation controller |

The current canter uses the touchdown order **right hind → right front + left hind → left front → flight**. Its planned cycle includes overlapping support, including a four-foot support interval. Configurations without that interval have not demonstrated reliable performance in the tests so far. Actual touchdown and flight can differ from the commanded schedule.

## Requirements

The development environment is **Ubuntu 20.04, ROS Noetic, and Gazebo Classic 11**, with an amd64 build configuration. These instructions assume ROS Noetic is already installed and configured.

Install the build tools and simulation dependencies:

```bash
sudo apt update
sudo apt install build-essential cmake git libeigen3-dev liblcm-dev libboost-all-dev \
  python3-rosdep ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
  ros-noetic-ros-control ros-noetic-ros-controllers \
  ros-noetic-robot-state-publisher ros-noetic-joint-state-publisher \
  ros-noetic-xacro ros-noetic-rviz
```

If you have not initialized rosdep on this machine, run `sudo rosdep init` once. Then run `rosdep update` as your normal user.

## Get the workspace and build

The repository itself is a catkin workspace. Clone it as the workspace root, **not inside another workspace's `src/` directory**:

```bash
git clone https://github.com/SamaviaJaffery/Unitree_Go1_Real_Word_Gait_Generation.git ~/unitree_ws
cd ~/unitree_ws
source /opt/ros/noetic/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro noetic -r -y
```

The Unitree sources are included in this repository; do not clone another copy of them into `src/`.

Check that [the controller build configuration](src/unitree_guide/unitree_guide/CMakeLists.txt) retains these settings:

```cmake
set(ROBOT_TYPE Go1)
set(PLATFORM amd64)
set(SIMULATION ON)
set(REAL_ROBOT OFF)
```

Build locally. A single-job build with reduced optimization and no debug symbols helps limit memory and disk use on a VM:

```bash
catkin_make -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_CXX_FLAGS_RELEASE='-O1 -DNDEBUG -g0' -j1 -l1
source devel/setup.bash
```

Use your actual workspace path if it differs from `~/unitree_ws`. Do not reuse another machine's `build/` or `devel/` outputs; rebuild from source. Avoid compiling while Gazebo is running on a memory-constrained VM.

## Run the simulation

### Terminal 1: Gazebo

```bash
cd ~/unitree_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
roslaunch unitree_guide gazeboSim.launch
```

Wait for the Go1 model and joint controllers to load. Gazebo displays the robot, but the locomotion executable must be started separately.

For a simulation without the Gazebo window, add `gui:=false` to the launch command.

### Terminal 2: keyboard controller

```bash
cd ~/unitree_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
./devel/lib/unitree_guide/junior_ctrl
```

Keep this terminal focused when entering commands. Run only one locomotion controller for the simulated robot.

### Start moving

1. Press `2` and wait for the robot to finish standing up.
2. Press `4` for trot, `6` for canter, or `7` for gallop.
3. Tap `w` to increase the forward command. Canter and gallop wait for a movement request, then uses a 0.4 s settling interval before activating its schedule.

Keyboard commands accumulate; releasing a key does not reset the command. For canter, each `w` press adds approximately **0.02 m/s** to the target. Two taps request 0.04 m/s; fifteen taps from zero request 0.30 m/s. The controller ramps toward the requested value, and actual velocity may differ.

## Keyboard controls

| Key | Action |
|---|---|
| `1` | Enter passive mode; the robot may settle or collapse rather than remain standing |
| `2` | Request fixed stand |
| `4` | Select trot from fixed stand |
| `6` | Select canter from fixed stand |
| `7` | Select gallop from fixed stand |
| `w` / `s` | Increase / decrease forward velocity command |
| `a` / `d` | Adjust lateral velocity command |
| `j` / `l` | Adjust yaw command in trot; canter and gallop currently hold their initial heading |
| Space | Clear velocity commands |
| `Ctrl+C` | Exit the controller |

**Space does not exit active canter or gallop.** These gaits remain active once started, even at zero velocity command. To stop cantering normally, press `2`; the current controller waits for an appropriate planned support window before returning to fixed stand. Use `1` to request passive mode when needed. Return to fixed stand before selecting a different gait. Gallop also gates its fixed-stand transition on planned four-foot support; if that condition is not reached, `2` will remain pending.

To finish a session, request fixed stand or passive as appropriate, stop the controller with `Ctrl+C`, then stop the Gazebo launch with `Ctrl+C`. If the robot falls, restart the simulation and controller before another trial.

## View the running robot in RViz

With Gazebo running, open another sourced terminal and run:

```bash
rviz
```

Set **Fixed Frame** to the robot's body frame, `trunk`, and add **RobotModel** and **TF** displays. The Gazebo launch already publishes the robot description and joint transforms. This view shows the articulated robot relative to its body frame; use Gazebo to observe world motion and ground contact.

The separate `go1_rviz.launch` in the description package publishes synthetic joint states for model inspection. Do not use it alongside the running simulation to visualize live joint feedback.

## Troubleshooting

| Symptom | What to check |
|---|---|
| Package or launch file cannot be found | Run `source devel/setup.bash` from the built workspace in that terminal. |
| `junior_ctrl` is missing | Check that `catkin_make` finished successfully and that you are at the workspace root. |
| Multiple packages named `unitree_legged_msgs` | Keep the included `CATKIN_IGNORE` in `src/unitree_ros_to_real/unitree_legged_msgs/`. The active message package is `src/unitree_legged_msgs/`. |
| Keyboard input has no effect | Focus the controller terminal, not Gazebo. Select fixed stand before selecting a gait. |
| `setProcessScheduler failed` | The process could not obtain real-time scheduling priority. This message alone does not terminate the controller. Check subsequent output and simulation feedback before diagnosing the shutdown. |
| XML-RPC connection refused | Check that the Gazebo/ROS launch is still running and both terminals use the same `ROS_MASTER_URI`. |
| The robot falls during debugging | A controller breakpoint does not pause Gazebo physics. Pause physics separately before inspecting a stopped controller. |
| VM slows down or runs low on space | Stop unused simulations, build with `-j1`, and check `df -h` and `free -h` before another build or test. |

To pause and resume Gazebo from a sourced terminal:

```bash
rosservice call /gazebo/pause_physics "{}"
rosservice call /gazebo/unpause_physics "{}"
```

Pausing also stops simulation time. Keyboard-driven state changes may not complete while the control loop is waiting for the simulation clock to advance.

## Where to find things

| Location | Contents |
|---|---|
| [unitree_guide](src/unitree_guide/unitree_guide/) | Locomotion controller, FSM, gait generation and force control |
| [State_Canter.cpp](src/unitree_guide/unitree_guide/src/FSM/State_Canter.cpp) | Canter control and state transitions |
| [State_Gallop.cpp](src/unitree_guide/unitree_guide/src/FSM/State_Gallop.cpp) | Gallop control and state transitions |
| [State_Canter.h](src/unitree_guide/unitree_guide/include/FSM/State_Canter.h) | Canter period, duty factor and leg offsets |
| [main.cpp](src/unitree_guide/unitree_guide/src/main.cpp) | Default shared gait configuration, including the optional crawl setting |
| [unitree_ros](src/unitree_ros/) | Robot models, Gazebo plugins and joint controllers |
| [unitree_legged_msgs](src/unitree_legged_msgs/) | ROS command and feedback messages |

Changing a source-level gait parameter requires rebuilding and restarting the controller. The active editor tab does not choose which gait runs.

## Acknowledgments and licenses

This project builds on Unitree Robotics' `unitree_guide`, robot descriptions and ROS simulation packages. See the included [unitree_guide license](src/unitree_guide/LICENSE), [unitree_ros license](src/unitree_ros/LICENSE), and notices in the bundled components for their respective terms.
