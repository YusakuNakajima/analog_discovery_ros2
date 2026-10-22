# AE activity over a sliding window, published periodically.
# Hits are placed in time by their capture stamp; captures missed during the
# dead time between captures are not counted.
std_msgs/Header header          # stamp = end of the window

float64 window_s
uint32 hits
float64 hit_rate_hz
float64 energy_rate_v2          # sum of hit energy_v2s / window_s (V^2)
float64 counts_rate_hz
float64 mean_peak_amplitude_v
float64 max_peak_amplitude_v
uint32 truncated_hits           # hits whose energy/duration are lower bounds
