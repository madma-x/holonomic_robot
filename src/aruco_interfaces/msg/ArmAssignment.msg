# Assignment of a detected tag to a gripper arm
uint8 arm_index    # 0-3 (physical arm numbering; 0 maps to the rightmost arm in the current robot layout)
uint32 tag_id
int32 track_id     # unique tracked instance id (-1 when unavailable/unassigned)
geometry_msgs/Pose tag_pose
bool assigned      # false if no tag was close enough for this arm
