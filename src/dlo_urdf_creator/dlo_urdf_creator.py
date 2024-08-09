import io
import numpy as np
from odio_urdf import *
import yourdfpy # for validating and visualizing the generated URDF

import materials

class DloURDFCreator:
    def __init__(self):
        self.N_global = 1 # Variable to keep track of the link number in the URDF model
    
    def create_dlo_urdf_equal_segment_length(self,
                                             length=1,
                                             radius=0.01,
                                             simplified_dlo_num_segments=20, 
                                             full_dlo_num_segments=40,
                                             full_dlo_holding_segment_ids=[5,34],
                                             environment_limits_xyz=[-1, 1, -1, 1, -1, 1],
                                             joint_angle_limits_xy_deg=[-90, 90, -180, 180],
                                             model_name="pole",
                                             base_link_name="base_link",
                                             tcp_link_name="tool0",
                                             center_link_name="center_link",
                                             holding_points_link_name_prefix= "pole_holder_link_",
                                             prism_joint_effort = 1,
                                             rev_joint_effort = 100,
                                             prism_joint_max_velocity = 1.0, # m/s
                                             rev_joint_max_velocity = 1.0, # rad/s
                                             visualize=False):
        
        """
        Compute the URDF string of the DLO with equal segment length.
        
        Args:
            length: The length of the DLO.
            radius: The radius of the DLO.
            simplified_dlo_num_segments: The number of segments of the simplified DLO.
            full_dlo_num_segments: The number of segments of the full DLO.
            full_dlo_holding_segment_ids: The list of segment IDs that have holding points.
            environment_limits_xyz: The limits of the environment in the x, y, and z directions for the initial end of the DLO.
            joint_angle_limits_xy_deg: The limits of the revolute joint angles in the x and y directions in degrees.
            model_name: The name of the DLO as robot in the URDF.
            base_link_name: The name of the base link.
            tcp_link_name: The name of the TCP link as the end tip of the DLO. Needed to be the last link in the URDF for the planner.
            center_link_name: The name of the center link. At the center of the DLO URDF model.
            holding_points_link_name_prefix: The prefix of the holding points link names. At the holding points of the DLO URDF model.
            visualize: A boolean to visualize the URDF model utilizing yourdfpy library.
            
        Returns:
            urdf_str: A string that represents the URDF of the DLO with equal segment length.
        """

        self.N_global = 1 # Variable to keep track of the link number in the URDF model
        self.prism_joint_effort = prism_joint_effort
        self.rev_joint_effort = rev_joint_effort
        self.prism_joint_max_velocity = prism_joint_max_velocity
        self.rev_joint_max_velocity = rev_joint_max_velocity

        print("N_global: ", self.N_global)   

        segment_length = length / simplified_dlo_num_segments
        
        # Build the robot structure
        urdf_obj = Robot(
            materials.materials,
            
            Link(base_link_name),
            Link(model_name+"_link_0"),
            
            Joint(Parent(link=base_link_name),
                Child(link=model_name+"_link_0"),
                Origin(xyz="0 0 0", rpy="0 0 0"),
                type="fixed",
                name= model_name+"_init_joint"),
            
            self._add_prismatic_joints(prefix=model_name, limits=environment_limits_xyz),
            
            self._add_revolute_joints_n_cylinders(prefix=model_name, 
                                            limits=joint_angle_limits_xy_deg,
                                            num_segments=simplified_dlo_num_segments,
                                            segment_r=radius,
                                            segment_l=segment_length, 
                                            tcp_link_name=tcp_link_name),
            
            self._add_holding_points_n_center(prefix=model_name,
                                        num_segments=simplified_dlo_num_segments,
                                        segment_l=segment_length,
                                        full_dlo_num_segments=full_dlo_num_segments,
                                        holding_segment_ids=full_dlo_holding_segment_ids,
                                        center_link_name=center_link_name,
                                        holding_points_link_name_prefix=holding_points_link_name_prefix),
            
            name = model_name
        )  
        
        urdf_str, is_valid_urdf = self._validate_and_visualize_urdf(str(urdf_obj), visualize=visualize)
        
        return urdf_str, is_valid_urdf

    def _prism_joint(self, N,
                     robot_name,
                     origin,
                     axis_str,
                     limits_meter, 
                     max_effort=1, 
                     max_velocity=1.0):
        N = int(N)
        ret = Joint(
            Parent(link=robot_name+"_link_"+str(N-1)),
            Child(link=robot_name+"_link_"+str(N)),
            Origin(origin),
            Axis(xyz=axis_str),
            Limit(lower=limits_meter[0], upper=limits_meter[1], effort=max_effort, velocity=max_velocity),
            type="prismatic",
            name= robot_name+"_prism_joint_"+str(N))
        return ret

    def _add_prismatic_joints(self, prefix, limits):
        # Create 3 links for the cartesian joint
        axes = [ [1, 0, 0], # x-axis
                [0, 1, 0], # y-axis
                [0, 0, 1]] # z-axis
        
        structure = []
        i = 0
        for axis in axes:
            link = Link(name=prefix+"_link_"+str(self.N_global))
            joint = self._prism_joint(self.N_global, prefix, [0, 0, 0], axis, 
                                      limits[2*i:2*i+2], self.prism_joint_effort, self.prism_joint_max_velocity)
            
            structure.append(link)
            structure.append(joint)
            self.N_global += 1
            i += 1
            
        ret = Group(*structure)
        return ret

    def rev_joint(self, N,
                  robot_name,
                  origin,
                  axis_str,
                  limits_deg, 
                  max_effort=100, 
                  max_velocity=1.0):
        N = int(N)
        ret = Joint(
            Parent(link=robot_name+"_link_"+str(N-1)),
            Child(link=robot_name+"_link_"+str(N)),
            Origin(origin),
            Axis(xyz=axis_str),
            Limit(lower=limits_deg[0]*np.pi/180, upper=limits_deg[1]*np.pi/180, effort=max_effort, velocity=max_velocity),
            type="revolute",
            name= robot_name+"_rev_joint_"+str(N))
        return ret

    def cylindrical_link(self, N,robot_name,material,r,l):
        """
            Most of the links are the same except for the passed in info.
            This function just allows grouping the important numbers better. 
        """
        N = str(N)
        ret = Link(
            Visual(
                Origin(xyz=[0, 0, l/2], rpy=[0, 0, 0]),
                Geometry(Cylinder(radius=r, length=l*0.98)),
                Material(material)),
            Collision(
                Origin(xyz=[0, 0, l/2], rpy=[0, 0, 0]),
                Geometry(Cylinder(radius=r, length=l*0.98)),
                Material(material)),
        name = robot_name+"_link_"+N)
        return ret

    def _add_revolute_joints_n_cylinders(self, prefix, 
                                        limits, 
                                        num_segments, 
                                        segment_r, 
                                        segment_l, 
                                        tcp_link_name): 
        # Create 2 links for the revolute joint
        axes = [ [1, 0, 0], # x-axis
                [0, 1, 0]] # y-axis
        structure = []
        
        for n in range(num_segments):
            
            if n == 0:
                i = 0
                
                link = Link(name=prefix+"_link_"+str(self.N_global))
                
                joint = self.rev_joint(self.N_global, prefix, [0, 0, 0], axes[i], [-180, 180], 
                                       self.rev_joint_effort, self.rev_joint_max_velocity)
                
                structure.append(link)
                structure.append(joint)
                
                self.N_global += 1
                i += 1
                
                # Attach the cylindrical link
                link = self.cylindrical_link(self.N_global, prefix, "Teal", segment_r, segment_l)
                
                joint = self.rev_joint(self.N_global, prefix, [0, 0, 0], axes[i], [-180, 180], 
                                       self.rev_joint_effort, self.rev_joint_max_velocity)
                
                structure.append(link)
                structure.append(joint)
                
                self.N_global += 1
                        
            else:
                i = 0
                
                link = Link(name=prefix+"_link_"+str(self.N_global))
                
                joint = self.rev_joint(self.N_global, prefix, [0, 0, segment_l], axes[i], 
                                       limits[2*i:2*i+2], self.rev_joint_effort, self.rev_joint_max_velocity)
                
                structure.append(link)
                structure.append(joint)
                
                self.N_global += 1
                i += 1
            
                # Attach the cylindrical link
                link = self.cylindrical_link(self.N_global, prefix, "Teal", segment_r, segment_l)
                
                joint = self.rev_joint(self.N_global, prefix, [0, 0, 0], axes[i], 
                                       limits[2*i:2*i+2], self.rev_joint_effort, self.rev_joint_max_velocity)
                
                structure.append(link)
                structure.append(joint)
                
                self.N_global += 1
            
            # Attach the end link and TCP link
            if n == num_segments - 1:
                # Attach the last link as the end tip of the DLO
                link = Link(name=prefix+"_link_"+str(self.N_global))
                
                joint = Joint(Parent(link=prefix+"_link_"+str(self.N_global-1)),
                            Child(link=prefix+"_link_"+str(self.N_global)),
                            Origin(xyz=[0, 0, segment_l], rpy=[0, 0, 0]),
                            type="fixed",
                            name= prefix+"_end_joint")
                
                structure.append(link)
                structure.append(joint)
                
                self.N_global += 1
                
                # Attach the TCP link
                link = Link(name=tcp_link_name)
                
                joint = Joint(Parent(link=prefix+"_link_"+str(self.N_global-1)),
                            Child(link=tcp_link_name),
                            Origin(xyz=[0, 0, 0], rpy=[0, 0, 0]),
                            type="fixed",
                            name= prefix+"_tcp_joint")
                
                structure.append(link)
                structure.append(joint)
                
        ret = Group(*structure)
        return ret

    def _add_holding_points_n_center(self, prefix, 
                                    num_segments, 
                                    segment_l, 
                                    full_dlo_num_segments,
                                    holding_segment_ids, 
                                    center_link_name, 
                                    holding_points_link_name_prefix):
        structure = []
        
        # Find the holding point attachment frames
        l = segment_l * num_segments # The length of the DLO
        
        holding_segment_ids = np.array(holding_segment_ids)
        
        segment_l_original = l / full_dlo_num_segments

        pos_of_holding_pts = segment_l_original * holding_segment_ids + segment_l_original / 2
        
        attachment_segments = pos_of_holding_pts // segment_l
        attachment_positions = pos_of_holding_pts % segment_l 
        
        attachment_global_frame_ids = attachment_segments * 2 + 5
        
        # Attach the holding points
        for i in range(len(attachment_segments)):
            frame_id = int(attachment_global_frame_ids[i])
            link_name = holding_points_link_name_prefix + str(holding_segment_ids[i])
            link = Link(name=link_name)
            
            joint = Joint(Parent(link=prefix+"_link_"+str(frame_id)),
                        Child(link=link_name),
                        Origin(xyz=[0, 0, attachment_positions[i]], rpy=[0, 0, 0]),
                        type="fixed",
                        name= prefix+"_holding_joint_"+str(i))
            
            structure.append(link)
            structure.append(joint)
            
        # Find the center attachment frame
        pos_of_center = l / 2
        attachment_segment = pos_of_center // segment_l
        attachment_position = pos_of_center % segment_l
        
        attachment_global_frame_id = int(attachment_segment * 2 + 5)
        
        # Attach the center link
        link = Link(name=center_link_name)
        
        joint = Joint(Parent(link=prefix+"_link_"+str(attachment_global_frame_id)),
                    Child(link=center_link_name),
                    Origin(xyz=[0, 0, attachment_position], rpy=[0, 0, 0]),
                    type="fixed",
                    name= prefix+"_center_joint")
        
        structure.append(link)
        structure.append(joint)
        
        ret = Group(*structure)
        return ret
            
    def _validate_and_visualize_urdf(self, urdf_str, visualize=False):
        is_valid_urdf = False
        urdf_model = None
        try:    
            print("\n--> Start validating the URDF model..\n")
            # Validate the URDF model with yourdfpy
            file_obj =  io.StringIO(urdf_str)
            urdf_model = yourdfpy.URDF.load(file_obj)
            
            if urdf_model.validate():
                is_valid_urdf = True
                print("URDF model is valid")
            else:
                print("URDF model is NOT valid")
        except Exception as e:
            print("Error validating the URDF model: ", e)
        
        try:
            if visualize:
                print("\n--> Start showing the URDF model..\n")
                if urdf_model is not None:
                    # Show the URDF model
                    urdf_model.show()
                else:
                    print("ERROR: No URDF model to visualize")
        except Exception as e:
            print("Error showing the URDF model: ", e)
                
        return urdf_str, is_valid_urdf

def save_urdf(urdf_str, output_file_path):
    if not (output_file_path == "" or output_file_path is None):
        try:
            # Save the urdf data to a file
            with open(output_file_path, "w") as file:
                file.write(urdf_str)
                print("Saved urdf data to file: ", output_file_path)
        except:
            print("Error saving urdf data to file: ", output_file_path)
    else:
        print("ERROR: No output file path provided")    
# ------------------------------------------------------------------------------
# Test cases     
    
def test_dlo_urdf_creator_equal_segment_length(visualize=False):
    print("------------------------------------------------------------------------------")
    print("Test case: test_dlo_urdf_creator_equal_segment_length")
    print("------------------------------------------------------------------------------")
    
    dlo_urdf_creator = DloURDFCreator()
    
    # Compute the URDF string
    urdf_str, is_valid_urdf = dlo_urdf_creator.create_dlo_urdf_equal_segment_length(visualize=visualize)

    # Print results
    print("\nURDF string from equal segment length:\n")
    print(urdf_str + "\n")
    
    # Save the URDF string to a file
    output_file_path = "dlo_urdf_equal_segment_length.urdf"
    save_urdf(urdf_str, output_file_path)
# ------------------------------------------------------------------------------
    
# Run the tests
if __name__ == "__main__":
    test_dlo_urdf_creator_equal_segment_length(visualize=True)