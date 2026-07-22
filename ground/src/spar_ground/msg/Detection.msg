# A camera-derived perception hit, already placed in the map frame. Any
# node that turns pixels into a point (the HSV anomaly detector, a YOLO
# node, a VLM) publishes this on perception/detections; label says what it
# found, so swapping detectors never means renaming a topic.
std_msgs/Header header
string label
geometry_msgs/Point point
