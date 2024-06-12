# deformable_description package

Created to be used for a planner that parses URDFs such as Tesseract.

### Some useful general commands when creating a description

To confirm the syntax validity

```
check_urdf <(xacro pole.xacro)
```

To convert xacro to urdf file

```
rosrun xacro xacro pole.xacro > ./pole.urdf
```

To visualize in RVIZ

```
roslaunch urdf_tutorial display.launch model:=./pole.urdf
```
