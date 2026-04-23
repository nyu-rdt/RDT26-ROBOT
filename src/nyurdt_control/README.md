this pkg is responsible for:
- starting i2c communication with teensy
- reading manual control inputs from the xbox controller

* twist_mux currently doesn't work yet

* Speed bucketing — Nav2 ramps velocity smoothly, so velocity_to_hex normalizes against MAX_LINEAR / MAX_ANGULAR (tune these to your actual hardware limits) and maps into your four tiers (25/50/75/100%). Your hex table is laid out so the tier is just a +offset from the base command, which makes that mapping clean.
Rotation priority — Nav2 on a diff-drive robot will often issue a pure-rotation command to face the goal before driving forward. The function prioritizes angular when |ang| >= |lin|, which matches that behavior. If your robot steers differently (e.g. simultaneous arc turns), flip that logic.
Change-only publishing — the bridge only forwards a new byte when the command changes, so you're not hammering the MCU at Nav2's 20 Hz publish rate with redundant identical bytes.
Dead-band — the DEAD = 0.05 threshold filters out near-zero Nav2 outputs at the end of a maneuver. Widen it if your MCU is sensitive to spurious motion commands.
One thing to watch: your AutonomyManager already calls publish_msg(0x50) (RIGHT_25) during stage_1 spin localization. If the bridge and the manager are both running, they'll fight over /robot/mcu_cmd. You'll want to gate the bridge — e.g. only activate it after stage_1 completes, or add a topic-level mutex using your existing SWITCH_AUTONOMOUS flag.
