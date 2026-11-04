# Box-ArUco camera detections, one entry per stack/zone currently in the
# face-mounted camera's FOV cone (see table_sim config/sim.yaml `camera:`).
# A name present here with count 0 means "visible and confirmed empty" -
# that's the point: it lets the strategy rule out a stack without driving
# up to it. A stack/zone absent from this array is simply out of view right
# now, not known to be anything.
string[] names     # stack/zone names, e.g. "hi_1", "Y_strip_hi"
int32[] counts     # true box count for each name, same order
