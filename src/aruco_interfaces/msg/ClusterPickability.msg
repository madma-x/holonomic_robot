# Pickability result for the best current pickup opportunity
std_msgs/Header header
int32 cluster_id              # currently fixed by aruco_manager; reserved for future grouping
ArmAssignment[4] arms
uint8 assigned_count
uint8 total_tags #total number of tags seen
bool is_pickable               # true if all residuals < consistency_threshold
geometry_msgs/Vector3 correction  # best-fit robot correction to apply (robot frame)
float32 correction_magnitude
