# Tests for the Cartesian Impedance Controller

This directory contains three test suites. They cover the base library (unit tests) and the ROS 2 integration layer (integration tests).

---

## Test suites

### 1. `base_tests` — Base library unit tests

**Source:** `base_tests.cpp`

Tests the `CartesianImpedanceController` C++ class directly, without any running ROS 2 system. 

**Requirements:** Eigen, gtest (no running ROS 2 system).

---

### 2. `ros_tests` — ROS 2 topic and parameter integration tests

**Source:** `ros_tests.cpp`  
**Launch file:** `ros_tests.launch.py`

Tests the running controller over ROS 2 interfaces.

**Requirements:** A running controller manager with `CartesianImpedance_trajectory_controller` active.

---

### 3. `ros_func_tests` — ROS 2 functional integration tests

**Source:** `ros_func_tests.cpp`  
**Launch file:** `ros_func_tests.launch.py`

Tests the controller's runtime behaviour over ROS 2 topics.

**Requirements:** A running simulation or real robot with the controller active, the robot initially positioned near y ≈ 0.3 m (as set up by the example simulation).

---

## Running the tests

Build and run with colcon:

```bash
cd ~/ros2_ws
colcon build --packages-select cartesian_impedance_controller
colcon test --packages-select cartesian_impedance_controller 
```

### Manually run Base library unit tests (no simulation needed)

Or build and invoke the binary directly:

```bash
colcon build --packages-select cartesian_impedance_controller
./build/cartesian_impedance_controller/base_tests
```

### Manually run Integration tests (simulation required)

**Step 1 – Start your simulation or robot stack** in a separate terminal. The controller manager must be running with `CartesianImpedance_trajectory_controller` active:

```
ros2 launch cartesian_impedance_controller minimal_mock_simulation.launch.py
```

**Step 2 – Run the tests**:

In another terminal, run the test executables:
```bash
./install/cartesian_impedance_controller/lib/cartesian_impedance_controller/ros_tests
./install/cartesian_impedance_controller/lib/cartesian_impedance_controller/ros_func_tests
```

### Filtering tests with `--gtest_filter`

```bash
# Only parameter existence test
./install/.../ros_tests --gtest_filter='ROSParameterTests.*'

# Only the wrench functional test
./install/.../ros_func_tests --gtest_filter='ROSFunctionalityTests.forceTests'
```

---

## Controller name

Both integration test executables default to the controller node name
`CartesianImpedance_trajectory_controller`. If your deployment uses a different
name, change the `ctrl_name` constant at the top of `ros_tests.cpp` and
`ros_func_tests.cpp` and rebuild.
