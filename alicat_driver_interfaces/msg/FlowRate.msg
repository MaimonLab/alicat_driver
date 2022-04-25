
# NOTE: the header msg is a ROS convention that contains timestamp and frame_id. It is not present in the original fictrac
std_msgs/Header header # stamp should be acquisition time of image.
                       # The ROS timestamp message has the two attributes: 
                            # sec (uint32)
                            # nanosec (uint32)
                       # frame_id (string), Hijacked to be frame_count at acquisition of the image (either camera or video_player). (ROS original intent is the point of reference for the camera. )

float32 flowrate # this should be the time difference between the time instance of interest and the acquisition time of the image 
float32 temperature
float32 pressure