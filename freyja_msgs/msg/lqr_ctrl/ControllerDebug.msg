# Debug message from controller.
# Often useful for observers, lqr_u is the 4-axis acceleration vector.
# State can be invalid due to time delay, or a dead source.

# flags:
uint8   CTRL_OK = 1
uint8   BIAS_EN = 2
uint8   FLAT_FF = 4
uint8   MASS_CR = 8

Header header
float32[4]  lqr_u
float32[4]  biasv
float32[7]  errv
uint8       flags
