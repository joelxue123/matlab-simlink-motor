# HPM EtherCAT Inertia Identification

This folder reads merged HPM EtherCAT trajectory CSV files and estimates
low-speed inertia from bidirectional torque pulses.

Use the HPM-side CSV time base:

```matlab
t = T.cmd_seq * 0.001;
```

Do not align identification by `rx_cycle`, because feedback frames may arrive in
batches while command sequence remains the stable experiment clock.

## Input

Expected CSV columns from `hpm_ecat_traj_stream`:

- `cmd_seq`
- `cmd_torque_nm`
- `feedback_valid`
- `position_rad`
- `speed_rad_s`
- `torque_nm`
- `error_code`

## Run

```matlab
run_csv = "/home/user/study/AI+MOTOR/HPM6E00EVK-RevC/ubuntu_ethercat_master/reports/hpm_inertia_pulse_low_torque_20260610_141812.csv";
R = analyze_hpm_inertia_pulse(run_csv);
```

The first low-torque run is expected to report almost no position motion, so it
is useful as a communication regression but not as a valid inertia fit.
