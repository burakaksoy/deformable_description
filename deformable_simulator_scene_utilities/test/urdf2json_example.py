import os
import io
import json

import numpy as np

import yourdfpy

# primitives_dir = "/path/to/primitives"
# primitives_dir = "/home/burak/tesseract_learning/tesseract/tesseract_support/urdf/deformable_description/meshes/primitives"
primitives_dir = "/home/burak/catkin_ws_deformable/src/dlo_simulator_stiff_rods/data/mesh/primitives"

def R2rot(R):
    """
    Recover k and theta from a 3 x 3 rotation matrix
    
        sin(theta) = | R-R^T |/2
        cos(theta) = (tr(R)-1)/2
        k = invhat(R-R^T)/(2*sin(theta))
        theta = atan2(sin(theta),cos(theta)
        
    :type    R: numpy.array
    :param   R: 3 x 3 rotation matrix    
    :rtype:  (numpy.array, number)
    :return: ( 3 x 1 k unit vector, rotation about k in radians)   
    
    """
    def invhat(khat):
        return np.array([(-khat[1,2] + khat[2,1]),(khat[0,2] - khat[2,0]),(-khat[0,1]+khat[1,0])])/2
    
    R1 = R-R.transpose()
    
    sin_theta = np.linalg.norm(R1)/np.sqrt(8)
    
    cos_theta = (np.trace(R) - 1.0)/2.0
    theta = np.arctan2(sin_theta, cos_theta)
    
    #Avoid numerical singularity
    if sin_theta < 1e-6:
               
        if (cos_theta > 0):
            return [0,0,1], 0
        else:
            B = (1.0/2.0) *(R + np.eye(3))
            k = np.sqrt([B[0,0], B[1,1], B[2,2]])
            if np.abs(k[0]) > 1e-6:
                k[1] = k[1] * np.sign(B[0,1] / k[0])
                k[2] = k[2] * np.sign(B[0,2] / k[0])
            elif np.abs(k[1]) > 1e-6:
                k[2] = k[2] * np.sign(B[0,2] / k[1])
            return k, np.pi
    
    k = invhat(R1)/(2.0*sin_theta)    
    return list(np.squeeze(k)), theta
    
def _urdf_to_json(urdf_model, primitives_dir="./"):
    # Validate the URDF model
    if urdf_model.validate():
        print("URDF model is valid")
    else:
        print("URDF model is not valid")
    # print("---------------------------------")
    
    # Show the URDF model
    urdf_model.show()

    json_data = {}
    json_data["Name"] = urdf_model.base_link

    # We will create a list of rigid bodies from the visuals in the urdf
    # with increasing id numbers
    rigid_bodies = []
    id = 1

    for link_name, link_obj in urdf_model.link_map.items():
        # print("link_name: ", link_name)
        # print("link_obj: ", link_obj)
        # print("")
        
        transform_base_link_to_link = urdf_model.get_transform(frame_to=link_name,
                                                        frame_from=urdf_model.base_link, 
                                                        collision_geometry=False) # 4x4 list
        transform_base_link_to_link = np.array(transform_base_link_to_link) # 4x4 numpy array
        
        if link_obj.visuals:
            for visual in link_obj.visuals:
                rb_dict = {}
                rb_dict["id"] = id
                
                # Find the transform from the base_link to the visual origin
                if visual.origin is not None:
                    transform_link_to_visual = np.array(visual.origin)
                    transform_base_link_to_visual = np.dot(transform_base_link_to_link, transform_link_to_visual)
                else:
                    transform_base_link_to_visual = transform_base_link_to_link
                
                # print("link visual transform to base_link: ", transform_base_link_to_visual)
                
                rotation_axis, rotation_angle = R2rot(transform_base_link_to_visual[:3, :3])
                rb_dict["rotationAxis"] = rotation_axis
                rb_dict["rotationAngle"] = float(rotation_angle)
                rb_dict["translation"] = list(transform_base_link_to_visual[:3, 3])
                
                # Find the Geometry file
                if visual.geometry.box:
                    geometry_file = f"{primitives_dir}/box.obj"
                    
                    # prints if the path is not resolved
                    geometry_file = yourdfpy.filename_handler_magic(geometry_file, "/") # 
                    rb_dict["geometryFile"] = geometry_file
                    
                    rb_dict["scale"] = list(map(float, visual.geometry.box.size))
                    rb_dict["collisionObjectScale"] = rb_dict["scale"]
                    
                if visual.geometry.cylinder:
                    geometry_file = f"{primitives_dir}/cylinder.obj"
                    
                    # prints if the path is not resolved
                    geometry_file = yourdfpy.filename_handler_magic(geometry_file, "/") # 
                    rb_dict["geometryFile"] = geometry_file
                    
                    radius = float(visual.geometry.cylinder.radius)
                    length = float(visual.geometry.cylinder.length)
                    rb_dict["scale"] = [radius, radius, length]
                    rb_dict["collisionObjectScale"] = rb_dict["scale"]
                    
                if visual.geometry.sphere:
                    geometry_file = f"{primitives_dir}/sphere.obj"
                    
                    # prints if the path is not resolved
                    geometry_file = yourdfpy.filename_handler_magic(geometry_file, "/") # 
                    rb_dict["geometryFile"] = geometry_file
                    
                    radius = float(visual.geometry.sphere.radius)
                    rb_dict["scale"] = [radius, radius, radius]
                    rb_dict["collisionObjectScale"] = rb_dict["scale"]
                    
                if visual.geometry.mesh:
                    geometry_file = visual.geometry.mesh.filename
                    
                    # prints if the path is not resolved
                    geometry_file = yourdfpy.filename_handler_magic(geometry_file, "/") 
                    rb_dict["geometryFile"] = geometry_file
                    
                    
                    if visual.geometry.mesh.scale is None:
                        rb_dict["scale"] = [1, 1, 1]
                    else:
                        rb_dict["scale"] = list(visual.geometry.mesh.scale)
                        
                    rb_dict["collisionObjectScale"] = rb_dict["scale"]
                
                # Fill the rest of the metadata with the default values
                rb_dict["isDynamic"] = 0
                rb_dict["density"] = 1.0
                rb_dict["velocity"] = [0.0, 0.0, 0.0]
                rb_dict["angularVelocity"] = [0.0, 0.0, 0.0]
                rb_dict["restitution"] = 0.0
                rb_dict["frictionStatic"] = 0.5
                rb_dict["frictionDynamic"] = 0.5
                rb_dict["comment"] = "collisionObjectFileName can contain the path of an SDF file or if it is empty, the simulator will generate an SDF using the mesh in the geometryFile"
                rb_dict["collisionObjectFileName"] = ""
                rb_dict["resolutionSDF"] = [50, 50, 50]
                rb_dict["invertSDF"] = 0
                

                rigid_bodies.append(rb_dict)
                id += 1
        else:
            # print("---- No visuals in link ----")
            pass
                
        # print("---------------------------------")
        
    json_data["RigidBodies"] = rigid_bodies

    # print("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
    # print("Created json_data: ")

    json_str = json.dumps(json_data, indent=4)
    # print(json_str)
    
    return json_str

def urdf_to_json(input_file_path,output_file_path="./output2.json"):
    urdf_model = yourdfpy.URDF.load(input_file_path)
    
    json_str = _urdf_to_json(urdf_model, primitives_dir)
    
    # Save the json_data to a file
    with open(output_file_path, "w") as file:
        file.write(json_str)
        print("Saved json_data to file: ", output_file_path)
    
def urdf_str_to_json(urdf_str,output_file_path="./output2.json"):
    file_obj =  io.StringIO(urdf_str)
    urdf_model = yourdfpy.URDF.load(file_obj)
    
    json_str = _urdf_to_json(urdf_model, primitives_dir)
    
    # Save the json_data to a file
    with open(output_file_path, "w") as file:
        file.write(json_str)
        print("Saved json_data to file: ", output_file_path)

## IMPORT FROM URDF STRING
urdf_str ="""<?xml version="1.0"?>
<robot name="a">
    <link name="corridor_base_link"/>

    <link name="link_ground_plane">
        <visual>
            <material name="">
                <color rgba="0.8 0.8 0.5 1"/>
            </material>
            <geometry>
                <box size="6 6 0.1"/>
            </geometry>
        </visual>
        <collision>
            <geometry>
                <box size="6 6 0.1"/>
            </geometry>
        </collision>
    </link>

    <joint name="joint_ground_plane" type="fixed">
        <origin xyz="0 0 -0.05"/>
        <parent link="corridor_base_link"/>
        <child link="link_ground_plane"/>
    </joint>

    <link name="outer_l_shaped_wall_origin_link"/>

    <joint name="outer_l_shaped_wall_origin_joint" type="fixed">
        <origin xyz="3 3 0"/>
        <parent link="corridor_base_link"/>
        <child link="outer_l_shaped_wall_origin_link"/>
    </joint>
    
    
    <link name="outer_along_x_wall_link">
        <visual>
            <origin xyz="0 0 2.2"/>
            <material name="">
                <color rgba="0.5 0.5 0.5 0.5"/>
            </material>
            <geometry>
                <box size="6 0.1 2.4"/>
            </geometry>
        </visual>
        <collision>
            <geometry>
                <box size="6 0.1 2.4"/>
            </geometry>
        </collision>
    </link>

    <joint name="outer_along_x_wall_link_joint" type="fixed">
        <origin xyz="-3 -0.05 1.2"/>
        <parent link="outer_l_shaped_wall_origin_link"/>
        <child link="outer_along_x_wall_link"/>
    </joint>
    
    <link name="outer_along_y_wall_link">
        <visual>
            <material name="">
                <color rgba="0.5 0.5 0.5 0.5"/>
            </material>
            <geometry>
                <box size="3 0.1 2.4"/>
            </geometry>
        </visual>
        <collision>
            <geometry>
                <box size="3 0.1 2.4"/>
            </geometry>
        </collision>
    </link>
    
    
    <joint name="outer_along_y_wall_link_joint" type="fixed">
        <origin xyz="-0.05 -1.5 1.2" rpy="3.14159 -3.14159 1.5708"/>
        <parent link="outer_l_shaped_wall_origin_link"/>
        <child link="outer_along_y_wall_link"/>
    </joint>
    
</robot>
"""

urdf_str_to_json(urdf_str=urdf_str, output_file_path="./output2.json")

# IMPORT DIRECTLY FROM FILE
# urdf_to_json(input_file_path="./output.urdf", output_file_path="./output3.json")
urdf_to_json(input_file_path="/home/burak/tesseract_learning/tesseract/tesseract_support/urdf/deformable_description/urdf/urdf_exported/urdf/l_shape_corridor.urdf", 
             output_file_path="./l_shape_corridor.json")











