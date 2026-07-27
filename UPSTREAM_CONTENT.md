# Upstream content policy

This repository keeps the original ROS1 U-Arm sources, mechanical CAD/STL files,
and the Armstrong ROS2 adapter used by this project.

The approximately 591 MB `src/simulation/mani_skill/` asset mirror is excluded
from this repository because it is not used by the Windows ZLink2 → Armstrong
hardware workflow. It remains available from the original upstream repository:

https://github.com/MINT-SJTU/LeRobot-Anything-U-Arm

This keeps a fresh Ubuntu clone small enough for practical robot deployment
without deleting any local CAD/STL files.
