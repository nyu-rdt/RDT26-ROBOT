---
# Motor Data
float32[8] motor_currents
int32 left_encoder
int32 right_encoder

# Excavation and Deposition
float32[4] load_cells
float32 total_mass
float32 conveyor_pos
float32 gate_pos

# Flags
bool heartbeat
bool battery_low_3s
bool battery_low_6s
bool motor_overcurrent
int32 fixes_attempted
