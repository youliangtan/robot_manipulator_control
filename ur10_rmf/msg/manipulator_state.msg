int8 gripper_state              # gripper's state, open/close/error
string arm_motion_state         # state of the motion, show the current motion group being executed
float32 arm_motion_progress     # indicate the progress of a group of motion, 0 means 0%, 1 means 100% completed

# Pose of eef
float32 x
float32 y
float32 z
float32 r
float32 p
float32 y