# Custom message for MTT-154 auxiliary commands

# Dead Man Switch / E-Stop
# true = pressed/active, false = released/estopped
bool dead_man_switch

# Analog Brake Command
# Range: 0.0 (no brake) to 1.0 (full brake)
float32 brake

# Winch Command
uint8 WINCH_NEUTRAL = 0
uint8 WINCH_IN = 1
uint8 WINCH_OUT = 2
uint8 winch_command

# Light light_state
int32 light_state  # 0=off, 1=on

bool winch_safety_button  # true = winch operations allowed, false = winch locked
