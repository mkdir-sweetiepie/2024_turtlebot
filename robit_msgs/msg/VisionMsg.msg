#line_tracking
int32 r_diff_pixel
int32 l_diff_pixel
int32 r_angle
int32 l_angle
bool r_line_info
bool l_line_info

##traffic
int32 traffic
##cross
bool cross_detect
int32 cross_info
##construct
bool construct_detect
int32 construct_info
##packing
bool parking_detect
int32 parking_info
##zigzag
bool zigzag_detect
bool zigzag_info
##gatebar
bool gatebar_detect
bool gatebar_info
##tunnel
int32 just_before_tunnel_num

#imu
int32 rel_angle
float64 rel_angle_ratio

#mission
int32[6] mission_sequence
