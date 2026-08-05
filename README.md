# UMRT ROS Template
ROS project repository template for the University of Manitoba Robotics Team.

New projects should be **forked** from this repo (not using this as a template, as that prevents template changes from
trickling down). Each new project must:
1. Fill in missing fields in package.in.xml
2. Fill in project name in CMakeLists.txt and Doxyfile
3. Go into `umrt-build` package settings and give the new repo read permission
4. Go into `umrt-apt-image` package settings and give the new repo read permission
5. Go into `UMRoboticsTeam` organisation secrets and add the new repo to:
   - `APT_DEPLOY_KEY`
   - `APT_SIGNING_KEY`
6. Copy the rulesets (branch protection rules) from a mature repository like
   [umrt-arm-firmware-lib](https://github.com/UMRoboticsTeam/umrt-arm-firmware-lib/)
7. Remove this notice and fill in below README template
8. Write something in mainpage.dox
9. Replace example files with real code, add source files to CMake targets, document it with Doxygen, and proceed

---
# Project Name
This library implements the automatic antenna controls for the University of Manitoba Robotics Team's automatic rotating antenna. This allows the antenna to automatically follow the rover as it moves around, allowing the rover to have the best signal strength possible at all times

[See the documentation](https://umroboticsteam.github.io/********** project-name **********/)
