# One triggered capture from an Analog Discovery oscilloscope input.
#
# header.stamp is the estimated host time of the trigger sample
# (ch*_v[trigger_index]): the host clock when the device was first seen done
# (before the data is read out), minus the post-trigger duration. Late by up to
# one poll period plus USB and scheduling latency. Use the
# device_* fields for precise spacing between captures (AD3: device counter,
# 10 ns at 100 MHz; AD2: host time).
std_msgs/Header header

uint64 capture_index       # captures published since the node started
float64 sample_rate_hz     # per channel, as read back from the device
uint32 trigger_index       # index of the trigger sample in ch1_v / ch2_v
bool auto_triggered        # true: the auto timeout fired; this is not a trigger event

float32[] ch1_v            # V
float32[] ch2_v            # V, empty when ch2 is disabled

# FDwfAnalogInStatusTime at the trigger: time = sec_utc + tick / ticks_per_second
uint32 device_trigger_sec_utc
uint32 device_trigger_tick
uint32 device_ticks_per_second
