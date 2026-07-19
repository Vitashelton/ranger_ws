# Mode-aware 动作 (单步), 与 engineering_decisions.md 附录A 对齐
uint8 MODE_ACKERMANN = 0
uint8 MODE_SPIN      = 1
uint8 MODE_STOP      = 2
uint8 MODE_OBLIQUE   = 3

std_msgs/Header header
uint8   mode
float32 param1    # v[m/s] (Ackermann/Oblique) 或 0 (Spin/Stop)
float32 param2    # delta[rad](Ackermann)/theta_dir[rad](Oblique)/omega[rad/s](Spin)/0(Stop)
