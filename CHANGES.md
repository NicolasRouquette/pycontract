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
* `colcon graph` now shows three packages: `pycontract`, `pycontract_examples`, `examples_ros2`.
* Build currently fails for `pycontract` until new ament_python configuration is recognised; next step is to re-run `colcon build` after these changes.
