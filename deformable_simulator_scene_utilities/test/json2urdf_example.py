import json
import numpy as np
from xml.etree.ElementTree import Element, SubElement, tostring
from xml.dom import minidom

def json_to_urdf(json_data):
    data = json.loads(json_data)
    
    # Create the root element of the URDF
    robot = Element('robot')
    robot.set('name', data['Name'])

    # Create an empty root link
    root_link = SubElement(robot, 'link', {'name': data['Name']})

    # Process each rigid body to create links and joints
    for body in data['RigidBodies']:
        # Extract file name without extension for link naming
        file_name = body['geometryFile'].split('/')[-1].split('.')[0]
        link_name = f"link_{file_name}_id_{body['id']}"

        # Create link element
        link = SubElement(robot, 'link', {'name': link_name})

        # Visual element
        visual = SubElement(link, 'visual')
        geometry_v = SubElement(visual, 'geometry')
        mesh_v = SubElement(geometry_v, 'mesh')
        mesh_v.set('filename', f"file://{body['geometryFile']}")
        mesh_v.set('scale', ' '.join(map(str, body['scale'])))

        # Collision element
        collision = SubElement(link, 'collision')
        geometry_c = SubElement(collision, 'geometry')
        mesh_c = SubElement(geometry_c, 'mesh')
        mesh_c.set('filename', f"file://{body['geometryFile']}")
        mesh_c.set('scale', ' '.join(map(str, body['collisionObjectScale'])))

        # Fixed joint connecting this link to the root link
        joint = SubElement(robot, 'joint', {'name': f'joint_{link_name}', 'type': 'fixed'})
        parent = SubElement(joint, 'parent', {'link': data['Name']})
        child = SubElement(joint, 'child', {'link': link_name})
        origin = SubElement(joint, 'origin')
        origin.set('xyz', ' '.join(map(str, body['translation'])))
        angle = body['rotationAngle']
        axis = body['rotationAxis']
        q = np.array([
            np.cos(angle / 2),
            np.sin(angle / 2) * axis[0],
            np.sin(angle / 2) * axis[1],
            np.sin(angle / 2) * axis[2]
        ])
        origin.set('rpy', ' '.join(map(str, [np.arctan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]**2 + q[2]**2)),
                                             np.arcsin(2*(q[0]*q[2] - q[3]*q[1])),
                                             np.arctan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]**2 + q[3]**2))])))

        # Add other properties as metadata
        for key, value in body.items():
            if key not in ['id', 'geometryFile', 'translation', 'rotationAxis', 'rotationAngle', 'scale', 'collisionObjectScale']:
                meta = SubElement(link, key)
                meta.text = str(value)

    # Convert the XML tree to a pretty-printed string
    rough_string = tostring(robot, 'utf-8')
    reparsed = minidom.parseString(rough_string)
    return reparsed.toprettyxml(indent="  ")


# Example usage
json_input = """
{
	"Name": "ClothCollisionScene1",
	"RigidBodies": [
		{
			"id": 1,
			"geometryFile": "/home/burak/catkin_ws_deformable/src/dlo_simulator_stiff_rods/data/mesh/cube_5.obj",
			"isDynamic": 0, 
			"density": 500, 
			"translation": [0,0,-0.05],
			"rotationAxis": [0, 0, 1.0],
			"rotationAngle": 0.0,
			"scale": [6.0, 6.0, 0.1],
			"velocity": [0,0,0],
			"angularVelocity": [0,0,0],
			"restitution" : 0.2,
			"frictionStatic" : 7.4,
			"frictionDynamic" : 7.2,
			"comment": "collisionObjectFileName can contain the path of an SDF file or if it is empty, the simulator will generate an SDF using the mesh in the geometryFile",
			"collisionObjectFileName": "../scenes/Cache/sphere.obj_50_50_50.csdf",
			"collisionObjectScale": [6.0, 6.0, 0.1],
			"resolutionSDF": [60,60,60],
			"invertSDF": 0
		},
		{
			"id": 2,
			"geometryFile": "/home/burak/catkin_ws_deformable/src/dlo_simulator_stiff_rods/data/mesh/cube_5.obj",
			"isDynamic": 0, 
			"density": 500, 
			"translation": [0,1,0.5],
			"rotationAxis": [0, 0, 1.0],
			"rotationAngle": 0.0,
			"scale": [1.5, 1.0, 1.0],
			"velocity": [0,0,0],
			"angularVelocity": [0,0,0],
			"restitution" : 0.0,
			"frictionStatic" : 3.4,
			"frictionDynamic" : 3.4,
			"comment": "collisionObjectFileName can contain the path of an SDF file or if it is empty, the simulator will generate an SDF using the mesh in the geometryFile",
			"collisionObjectFileName": "../scenes/Cache/sphere.obj_50_50_50.csdf",
			"collisionObjectScale": [1.5, 1.0, 1.0],
			"resolutionSDF": [50,50,50],
			"invertSDF": 0
		},
		{
			"id": 3,
			"geometryFile": "/home/burak/catkin_ws_deformable/src/dlo_simulator_stiff_rods/data/mesh/tent_pole_grommet.obj",
			"isDynamic": 0, 
			"density": 500, 
			"translation": [-1,-1,0.15],
			"rotationAxis": [0,1,0],
			"rotationAngle": 0.0,
			"scale": [1,1,1],
			"velocity": [0,0,0],
			"angularVelocity": [0,0,0],
			"restitution" : 0.0,
			"frictionStatic" : 0.0,
			"frictionDynamic" : 0.0,
			"comment": "collisionObjectFileName can contain the path of an SDF file or if it is empty, the simulator will generate an SDF using the mesh in the geometryFile",
			"collisionObjectFileName": "/home/burak/catkin_ws_deformable/src/dlo_simulator_stiff_rods/data/mesh/tent_pole_grommet.obj_80_80_80.csdf",
			"collisionObjectScale": [1,1,1],
			"resolutionSDF": [250,250,250],
			"invertSDF": 0
		},
		{
			"id": 4,
			"geometryFile": "/home/burak/catkin_ws_deformable/src/dlo_simulator_stiff_rods/data/mesh/tent_pole_grommet.obj",
			"isDynamic": 0, 
			"density": 500, 
			"translation": [1,-1,0.15],
			"rotationAxis": [0,-1, 0],
			"rotationAngle": 0.0,
			"scale": [1,1,1],
			"velocity": [0,0,0],
			"angularVelocity": [0,0,0],
			"restitution" : 0.0,
			"frictionStatic" : 0.0,
			"frictionDynamic" : 0.0,
			"comment": "collisionObjectFileName can contain the path of an SDF file or if it is empty, the simulator will generate an SDF using the mesh in the geometryFile",
			"collisionObjectFileName": "/home/burak/catkin_ws_deformable/src/dlo_simulator_stiff_rods/data/mesh/tent_pole_grommet.obj_80_80_80.csdf",
			"collisionObjectScale": [1,1,1],
			"resolutionSDF": [250,250,250],
			"invertSDF": 0
		}

	]
}
"""
print(json_to_urdf(json_input))
