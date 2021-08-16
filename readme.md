
# Planar 3DOF manipulator robot

this project implements a 3D planar robot in urdf. It also contain a launch with a *robot_state_publisher* and a *joint_state_publisher_gui*

This urdf create a planar robot with the next chain:
```
robot name is: planar_3dof
---------- Successfully Parsed XML ---------------
root Link: world has 1 child(ren)
    child(1):  base_link
        child(1):  link_1
            child(1):  link_2
                child(1):  gripper
                    child(1):  end

```

This is an adaptation of an old example of ros industrial tutorial(before [docs](https://industrial-training-master.readthedocs.io/) ) about how to create and urdf files

![](robotplanar.PNG)

where, $l_1=l_2=0.5$ y $l_3=0.18$ 

TODO: put the instruction about how to create the urdf



To execute this package please add this to the .bashrc or to the specific console ([Show basic URDF model in Rviz2](https://answers.ros.org/question/348984/show-basic-urdf-model-in-rviz2/) )


echo "export LC_NUMERIC="en_US.UTF-8"" >> ~/.bashrc

to test make
```
cd workspace
colcon build
ros2 launch lesson_urdf view_robot_launch.py
```



