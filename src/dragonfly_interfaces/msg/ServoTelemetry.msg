# valid means a successful bus read, not calibrated torque or simultaneous ADC sampling.
uint8 id
bool valid
uint64 read_completed_us
int32 position_raw
int32 speed_raw
int32 current_raw
int32 load_raw
uint8 status
float64 voltage_v
float64 temperature_c
float64 current_a
float64 torque_estimate_nm

uint8 response_error
