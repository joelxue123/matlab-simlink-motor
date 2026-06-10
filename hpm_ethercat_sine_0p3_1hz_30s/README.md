# HPM EtherCAT sine test: 0.3 rad, 1 Hz, 30 s

## Test setup

- Date: 2026-06-09
- Board: HPM6E00EVK RevC
- Motor target ID: `0x04`
- Host interface: `enp129s0`
- EtherCAT period: `1000 us`
- Trajectory mode: sine
- Amplitude: `0.3 rad`
- Frequency: `1 Hz`
- Duration: `30 s`
- MIT gains: `kp = 12`, `kd = 0.2`
- Torque feedforward: `0 Nm`
- Host realtime: `SCHED_FIFO priority 80`, `taskset -c 21`

## Source files

- CSV: `data/hpm_traj_sine_amp0p3_freq1hz_kp12_kd0p2_30s_20260609_185429.csv`
- Run log: `data/hpm_traj_sine_amp0p3_freq1hz_kp12_kd0p2_30s_20260609_185429.txt`

## Command used

```bash
cd /home/user/study/AI+MOTOR/HPM6E00EVK-RevC
printf '1\n' | sudo -S ip link set enp129s0 up
printf '1\n' | sudo -S taskset -c 21 \
  ./ubuntu_ethercat_master/build/hpm_ecat_traj_stream \
  enp129s0 sine 0x04 30000 1000 0.3 1 12 0.2 0 \
  ubuntu_ethercat_master/reports/hpm_traj_sine_amp0p3_freq1hz_kp12_kd0p2_30s_20260609_185429.csv
```

## Communication result

```text
loops=30001
points_sent=30000
tx_packets=29970
wkc_fail=0
rx_bytes=599960
frames=29998
crc_ok=29998
crc_bad=0
control_frames=29998
rx_buffer_drops=0
enqueued=30000
played=30000
underrun=3
tx_busy=0
```

Timing:

```text
late_us: min=0 avg=4 p50=4 p90=6 p99=11 p99.9=39 max=225
exchange_us: min=47 avg=631 p50=676 p90=690 p99=824 p99.9=933 max=1160
```

## MATLAB processing

Run:

```matlab
analyze_hpm_sine_0p3_1hz_30s
```

The script reads the CSV, reconstructs the nominal sine command, estimates the 1 Hz response amplitude and phase delay, prints metrics, and saves a figure to `output/hpm_sine_0p3_1hz_30s_analysis.png`.

Processed result:

```text
samples: 29998
frame_index: 1 .. 29998
frame_index non-continuous steps: 0
cycle: 2 .. 29998
cycle duplicate steps: 247
cycle gap events: 246
cycle missing count: 246
nonzero error_code samples: 0
position range: -0.297932 .. 0.297932 rad
speed range: -1.890593 .. 2.029755 rad/s
torque range: -0.046159 .. 0.129625 Nm
command position range: -0.300000 .. 0.300000 rad
command speed range: -1.884956 .. 1.884956 rad/s
nominal error RMS: 0.020554 rad
nominal error max abs: 0.033952 rad
estimated 1Hz amp: 0.299164 rad
estimated phase: -0.096708 rad
estimated delay: 15.391 ms
estimated offset: 0.000008 rad
```

CSV conclusion:

- No discontinuity was found in `frame_index`; the received motor frames inside the CSV are continuous.
- `cycle` has duplicate and gap steps because the host reassembles a UART byte stream. One EtherCAT cycle may contain zero, one, or two complete motor frames.
- Rows with the same `cycle` are not duplicated samples. Their `frame_index` values are different and their position/speed/torque values continue changing.
- The run log shows `points_sent=30000`, `played=30000`, `frames=29998`, `crc_bad=0`. The two-frame difference is most likely caused by the host stopping immediately after the stop packet instead of draining the last UART replies.
- The commanded sine was generated as `pos = 0.3 * sin(2*pi*1*t)` and `vel = 2*pi*1*0.3*cos(2*pi*1*t)`.
