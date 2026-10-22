# Acoustic-emission hit features of one ScopeCapture (analog_discovery/ae_features.py).
# header is copied from the capture (stamp = estimated trigger time).
# Amplitudes are at the sensor amplifier output; its gain is not removed.
std_msgs/Header header

uint64 capture_index
float64 threshold_v             # detection threshold used for the features below

float64 peak_amplitude_v        # V, largest |v| after the trigger (pre-trigger mean removed)
float64 rise_time_s             # first threshold crossing -> peak
float64 duration_s              # first -> last threshold crossing (lower bound if truncated)
uint32 counts                   # upward crossings of +threshold
float64 energy_v2s              # integral of v^2 over the duration, V^2 s (lower bound if truncated)
float64 rms_v                   # V, over the duration
float64 peak_frequency_hz
float64 centroid_frequency_hz
float64 noise_rms_v             # V, pre-trigger samples
bool truncated                  # the hit outlasted the capture
