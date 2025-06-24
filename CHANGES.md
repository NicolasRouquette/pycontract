# Change Log

## 2025-06-24

### Project restructuring
* Moved source code into `src/` layout and staged git renames for former top-level directories (`examples`, `examples-ros2`, `pycontract`).

### New build / package files
* **src/examples**
  * Added `CMakeLists.txt` (assistant) and updated it to install `run_monitor.py` only (user).
  * Added `package.xml` (assistant) and updated maintainer + switched to `ament_python` (user / assistant).
* **src/examples_ros2**
  * Added `pycontract` dependency to `package.xml` (assistant).
  * Updated maintainers in `package.xml` (user).
* **src/pycontract**
  * Added `setup.py` for ament_python build (assistant).
  * Converted `package.xml` to use `ament_python`; removed ROS 2 specific deps and added a single `<export><build_type>ament_python</build_type></export>` section (assistant).

### Repository meta
* Updated `.gitignore` to exclude `build`, `install`, and `log` directories (user).
* Created (and later removed) top-level `CMakeLists.txt` while investigating colcon graph visibility (assistant then user deleted).

### Build status
* Converted **src/examples_ros2** from `ament_python` to `ament_cmake`:
  * Added `CMakeLists.txt` to generate custom messages and install scripts.
  * Updated `package.xml` to use `ament_cmake`, add `rosidl_default_generators/runtime` deps.
  * Removed need for `setup.py`; build error fixed.
* Added `setup.py` and `resource/pycontract_examples` to **src/examples** so that package `pycontract_examples` builds correctly with `ament_python`.
* `colcon graph` now shows three packages: `pycontract`, `pycontract_examples`, `examples_ros2`.
* Added `resource/pycontract` file required by ament to register the package, fixing the previous colcon build error.
* Re-run `colcon build` should now succeed for all packages.
