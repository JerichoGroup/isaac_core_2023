# x1, y1, x2, y2 - bbox boundrys in pixels on the axis
# lat, lon, alt - location of the obejct in the world
# roll, pitch, yaw - local orientation of the object (euler angels XYZ notation)
# distance x/y/z - the local distance between the camera and object on the axis in meters

string target_name
bool in_frame
bool is_visible
int32 x1
int32 y1
int32 x2
int32 y2
float32 lat
float32 lon
float32 alt
float32 roll
float32 pitch
float32 yaw
float32 distance_x
float32 distance_y
float32 distance_z
