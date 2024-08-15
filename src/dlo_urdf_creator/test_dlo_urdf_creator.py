from dlo_urdf_creator import DloURDFCreator
# ------------------------------------------------------------------------------
# Test cases     
    
def test_dlo_urdf_creator_equal_segment_length(visualize=False):
    print("------------------------------------------------------------------------------")
    print("Test case: test_dlo_urdf_creator_equal_segment_length")
    print("------------------------------------------------------------------------------")
    
    dlo_urdf_creator = DloURDFCreator()
    
    # Compute the URDF string
    urdf_str, is_valid_urdf, allowed_collision_pairs = dlo_urdf_creator.create_dlo_urdf_equal_segment_length(visualize=visualize)

    # Print results
    print("\nURDF string from equal segment length:\n")
    print(urdf_str + "\n")
# ------------------------------------------------------------------------------
    
# Run the tests
if __name__ == "__main__":
    test_dlo_urdf_creator_equal_segment_length(visualize=True)