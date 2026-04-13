# Computed by TelemetryProcessor
float32 total_mass
float32 mass_flow
float32 slip_score
float32 motor_currents
float32 left_encoder
float32 right_encoder
float32 load_cells
float32 conveyor_pos
float32 gate_pos


# Locomotion flags
bool slip_warn
bool slip_severe
bool stall_risk
bool digging_in

# Excavation flags
bool ladder_jam_risk
bool empty_cut
bool exc_pos_error

# Deposition flags
bool gate_not_closed
bool gate_stall
bool depo_empty
bool depo_flow_blocked

bool heartbeat
bool batter_low_3s
bool batter_low_6s
bool motor_overcurrent
