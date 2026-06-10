# HPM F2 feedback sine test: 0.3 rad, 1 Hz, 30 s

## Test setup

- Date: 2026-06-09
- Feedback transport: HPM `F2` structured feedback, 2 records per TxPDO, window retransmit
- Motor target ID: `0x04`
- EtherCAT interface: `enp129s0`
- Period: `1000 us`
- Command: sine, `0.3 rad`, `1 Hz`, `30 s`
- MIT gains: `kp = 12`, `kd = 0.2`
- Torque feedforward: `0 Nm`

## Files

- CSV: `data/hpm_motorfb_f2win_sine_amp0p3_freq1hz_kp12_kd0p2_30s_20260609_195539.csv`
- Run log: `data/hpm_motorfb_f2win_sine_amp0p3_freq1hz_kp12_kd0p2_30s_20260609_195539.txt`
- MATLAB: `analyze_hpm_f2win_sine_0p3_1hz_30s.m`

## Communication result

```text
points_sent=30000
enqueued=30000
played=30000
wkc_fail=0
frames=30000
crc_ok=30000
crc_bad=0
feedback_frames=30000
pending=0
pending_missing_samples=0
fb_overflow_samples=0
cmd_seq=0..29999
missing_cmd=0
dup_cmd=0
error_code_nonzero=0
```

`rx_cycle` is not the analysis time axis. It can repeat because one F2 TxPDO can carry two feedback records, and repeated windows are filtered by `feedback_seq`.

## MATLAB

Run:

```matlab
analyze_hpm_f2win_sine_0p3_1hz_30s
```

The script uses `cmd_seq * 1 ms` as the time axis and saves a figure plus text summary under `output/`.

## Processed result

```text
samples=30000
cmd_seq range=0..29999
missing cmd count=0
duplicate cmd count=0
nonzero error_code samples=0
position range=-0.298028..0.297932 rad
speed range=-1.888151..1.988251 rad/s
torque range=-0.042954..0.113603 Nm
command position range=-0.300000..0.300000 rad
command speed range=-1.884956..1.884956 rad/s
nominal error RMS=0.022127 rad
nominal error max abs=0.033116 rad
estimated 1Hz amp=0.299167 rad
estimated phase=-0.104350 rad
estimated delay=16.608 ms
estimated offset=0.000014 rad
```

Generated outputs:

```text
output/hpm_f2win_sine_0p3_1hz_30s_analysis.png
output/hpm_f2win_sine_0p3_1hz_30s_summary.txt
```
