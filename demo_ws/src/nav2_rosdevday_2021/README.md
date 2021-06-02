# nav2_rosdevday_2021
Custom scripts, packages, and notebooks for the ROS Dev Day 2021 talk

This contains:
- Demo for running Nav2 in the AWS Warehouse World
- Demo for running Nav2 on a non-TB3 industrial robot
- Demo for Keepout Zones and Speed Restricted Zones
- Demo code for working with Nav2 in python3 simple autonomy application
- A Jupyter notebook with instructions for a Nav2 walkthrough of these features

This demonstration is overall a reasonable baseline for how to work with Nav2 with a non-default setup (e.g. non-Turtlebot3 in a non-sandbox world).

# Steve Notes

Must source both aws warehouse and TB3 for plugins

```
export GAZEBO_MODEL_PATH=/home/steve/nav2_rosdevday_2021/demo_ws/src/aws-robomaker-small-warehouse-world/models:/home/steve/Documents/navigation2_ws/turtlebot3_simulations/turtlebot3_gazebo/models
```
