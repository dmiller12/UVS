## Common ROS msgs and srvs for the Barrett WAM 
Clone and compile this package in the source folder of your ROS catkin workspace to include in custom WAM packages. <br />
These msgs and srvs are the same that are advertised by the wam_bringup launch file run on the WAM. <br />
### Add to your package.xml file 
```
  <build_depend>wam_msgs</build_depend>
  <build_depend>wam_srvs</build_depend>
  <run_depend>wam_msgs</run_depend>
  <run_depend>wam_srvs</run_depend>
```
### Add to your CMakeLists.txt file 
```
find_package(catkin REQUIRED COMPONENTS
  wam_msgs
  wam_srvs
)
```


