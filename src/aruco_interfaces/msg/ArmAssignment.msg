# Assignment of a detected tag to a gripper arm
uint8 arm_index    # 0-3 (left to right on Y axis, 5cm spacing)
uint32 tag_id
bool assigned      # false if no tag was close enough for this arm
