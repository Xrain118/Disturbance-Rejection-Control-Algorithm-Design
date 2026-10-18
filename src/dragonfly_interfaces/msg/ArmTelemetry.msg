# Header is host receive time. Device timestamps have an independent clock per arm.
std_msgs/Header header
string device_id
uint32 boot_id
uint32 sequence
uint64 device_time_us
float64 target_rate_hz
uint32 scan_duration_us
uint32 overruns
uint32 dropped_frames
bool legacy
string estimate_method
string calibration_status
string parameter_version
ServoTelemetry[] servos
string raw_json
