# Pickability result for one cluster of object tags
std_msgs/Header header
int32 cluster_id
ArmAssignment[4] arms
uint8 assigned_count
bool is_pickable               # true if all residuals < consistency_threshold
geometry_msgs/Vector3 correction  # best-fit robot correction to apply (robot frame)
float32 correction_magnitude
