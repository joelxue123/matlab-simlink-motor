%% Build shared data dictionary for reusable motor-control modules
%
% This dictionary is the team-facing contract for library reuse. It contains
% common AliasType, Bus, and default parameter entries used by the reusable
% controller modules.

clear; clc;

script_dir = fileparts(mfilename('fullpath'));
previous_dir = pwd;
cleanup_dir = onCleanup(@() cd(previous_dir));
cd(script_dir);

cfg = customer_interface_config();
cfg.dictionaryFile = fullfile(script_dir, cfg.dictionaryName);
params = default_motor_control_params();

close_open_data_dictionaries();
dd = recreate_data_dictionary(cfg);
design_data = getSection(dd, 'Design Data');

upsert_alias_type(design_data, cfg.realTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.voltageTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.angleTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.currentTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.speedTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.torqueTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.dutyTypeName, cfg.floatBaseType, cfg.typeHeaderFile);

upsert_alias_type(design_data, cfg.currentPiRealTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.currentPiCurrentTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.currentPiVoltageTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.currentPiSpeedTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.currentPiGainTypeName, cfg.floatBaseType, cfg.typeHeaderFile);

upsert_alias_type(design_data, cfg.speedPiRealTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.speedPiSpeedTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.speedPiCurrentTypeName, cfg.floatBaseType, cfg.typeHeaderFile);
upsert_alias_type(design_data, cfg.speedPiGainTypeName, cfg.floatBaseType, cfg.typeHeaderFile);

upsert_bus_type(design_data, cfg.motorInputBusName, cfg.typeHeaderFile, { ...
    {'ia', cfg.currentTypeName}; ...
    {'ib', cfg.currentTypeName}; ...
    {'ic', cfg.currentTypeName}; ...
    {'theta_e', cfg.angleTypeName}});

upsert_bus_type(design_data, cfg.motorDqBusName, cfg.typeHeaderFile, { ...
    {'i_alpha', cfg.currentTypeName}; ...
    {'i_beta', cfg.currentTypeName}; ...
    {'id', cfg.currentTypeName}; ...
    {'iq', cfg.currentTypeName}});

upsert_bus_type(design_data, cfg.openLoopCmdBusName, cfg.typeHeaderFile, { ...
    {'vd', cfg.voltageTypeName}; ...
    {'vq', cfg.voltageTypeName}; ...
    {'theta_e', cfg.angleTypeName}; ...
    {'vdc', cfg.voltageTypeName}});

upsert_bus_type(design_data, cfg.phaseDutyBusName, cfg.typeHeaderFile, { ...
    {'da', cfg.dutyTypeName}; ...
    {'db', cfg.dutyTypeName}; ...
    {'dc', cfg.dutyTypeName}});

upsert_bus_type(design_data, cfg.plantFeedbackBusName, cfg.typeHeaderFile, { ...
    {'ia', cfg.currentTypeName}; ...
    {'ib', cfg.currentTypeName}; ...
    {'ic', cfg.currentTypeName}; ...
    {'wm', cfg.speedTypeName}; ...
    {'theta_m', cfg.angleTypeName}; ...
    {'theta_e', cfg.angleTypeName}});

upsert_bus_type(design_data, cfg.currentPiInputBusName, cfg.typeHeaderFile, { ...
    {'id_ref', cfg.currentPiCurrentTypeName}; ...
    {'iq_ref', cfg.currentPiCurrentTypeName}; ...
    {'id_meas', cfg.currentPiCurrentTypeName}; ...
    {'iq_meas', cfg.currentPiCurrentTypeName}; ...
    {'omega_e', cfg.currentPiSpeedTypeName}; ...
    {'vdc', cfg.currentPiVoltageTypeName}});

upsert_bus_type(design_data, cfg.currentPiOutputBusName, cfg.typeHeaderFile, { ...
    {'vd_ref', cfg.currentPiVoltageTypeName}; ...
    {'vq_ref', cfg.currentPiVoltageTypeName}});

upsert_bus_type(design_data, cfg.speedPiInputBusName, cfg.typeHeaderFile, { ...
    {'wm_ref', cfg.speedPiSpeedTypeName}; ...
    {'wm_meas', cfg.speedPiSpeedTypeName}; ...
    {'iq_limit', cfg.speedPiCurrentTypeName}});

upsert_bus_type(design_data, cfg.speedPiOutputBusName, cfg.typeHeaderFile, { ...
    {'iq_ref', cfg.speedPiCurrentTypeName}});

upsert_parameter(design_data, 'Kp_id', params.Kp_id, cfg.currentPiGainTypeName);
upsert_parameter(design_data, 'Ki_id', params.Ki_id, cfg.currentPiGainTypeName);
upsert_parameter(design_data, 'Kaw_id', params.Kaw_id, cfg.currentPiGainTypeName);
upsert_parameter(design_data, 'Kp_iq', params.Kp_iq, cfg.currentPiGainTypeName);
upsert_parameter(design_data, 'Ki_iq', params.Ki_iq, cfg.currentPiGainTypeName);
upsert_parameter(design_data, 'Kaw_iq', params.Kaw_iq, cfg.currentPiGainTypeName);
upsert_parameter(design_data, 'VLimitRatio', params.VLimitRatio, cfg.currentPiGainTypeName);

upsert_parameter(design_data, 'Kp_speed', params.Kp_speed, cfg.speedPiGainTypeName);
upsert_parameter(design_data, 'Ki_speed', params.Ki_speed, cfg.speedPiGainTypeName);
upsert_parameter(design_data, 'Kaw_speed', params.Kaw_speed, cfg.speedPiGainTypeName);
upsert_parameter(design_data, 'IqLimitDefault', params.IqLimitDefault, cfg.speedPiCurrentTypeName);

upsert_entry(design_data, 'simcfg', params.simcfg);
upsert_entry(design_data, 'current_pi_simcfg', params.current_pi_simcfg);
upsert_entry(design_data, 'speed_pi_simcfg', params.speed_pi_simcfg);
upsert_entry(design_data, 'inverter', params.inverter);
upsert_entry(design_data, 'openloop', params.openloop);

saveChanges(dd);
close(dd);

fprintf('Built shared motor-control data dictionary:\n  %s\n', cfg.dictionaryFile);

function cfg = customer_interface_config()
cfg.dictionaryName = 'motor_control_interface.sldd';
cfg.typeHeaderFile = 'motor_control_types.h';

cfg.realTypeName = 'T_MotorFloat';
cfg.voltageTypeName = 'T_MotorVoltage';
cfg.angleTypeName = 'T_MotorAngle';
cfg.currentTypeName = 'T_MotorCurrent';
cfg.speedTypeName = 'T_MotorSpeed';
cfg.torqueTypeName = 'T_MotorTorque';
cfg.dutyTypeName = 'T_DutyRatio';

cfg.currentPiRealTypeName = 'T_CurrentPiFloat';
cfg.currentPiCurrentTypeName = 'T_CurrentPiCurrent';
cfg.currentPiVoltageTypeName = 'T_CurrentPiVoltage';
cfg.currentPiSpeedTypeName = 'T_CurrentPiSpeed';
cfg.currentPiGainTypeName = 'T_CurrentPiGain';

cfg.speedPiRealTypeName = 'T_SpeedPiFloat';
cfg.speedPiSpeedTypeName = 'T_SpeedPiSpeed';
cfg.speedPiCurrentTypeName = 'T_SpeedPiCurrent';
cfg.speedPiGainTypeName = 'T_SpeedPiGain';

cfg.motorInputBusName = 'motor_t';
cfg.motorDqBusName = 'motor_dq_t';
cfg.openLoopCmdBusName = 'open_loop_cmd_t';
cfg.phaseDutyBusName = 'phase_duty_t';
cfg.plantFeedbackBusName = 'plant_feedback_t';
cfg.currentPiInputBusName = 'current_pi_input_t';
cfg.currentPiOutputBusName = 'current_pi_output_t';
cfg.speedPiInputBusName = 'speed_pi_input_t';
cfg.speedPiOutputBusName = 'speed_pi_output_t';

cfg.floatBaseType = 'single';
end

function p = default_motor_control_params()
p.simcfg.stop_time = 0.060;
p.simcfg.Ts_plant = 25e-6;
p.simcfg.Ts_ctrl = 50e-6;
p.simcfg.Ts_speed = 100e-6;

p.current_pi_simcfg.Ts_ctrl = p.simcfg.Ts_ctrl;
p.speed_pi_simcfg.Ts_speed = p.simcfg.Ts_speed;

p.inverter.Vdc = 68;
p.inverter.load_torque = 0;
p.inverter.current_limit = 15;

pole_pairs = 10;
back_emf_vrms_per_krpm = 17.03;
line_to_line_resistance = 0.4267;
line_to_line_inductance = 0.53e-3;
Rs = line_to_line_resistance / 2;
Ld = line_to_line_inductance / 2;
psi_f = back_emf_vrms_per_krpm / ...
    (sqrt(3/2) * pole_pairs * (1000 * 2 * pi / 60));
J = 2.5e-4;
torque_constant = 1.5 * pole_pairs * psi_f;

current_bandwidth_rad_s = 2 * pi * 800;
p.Kp_id = single(Ld * current_bandwidth_rad_s);
p.Ki_id = single(Rs * current_bandwidth_rad_s);
p.Kaw_id = single(400);
p.Kp_iq = p.Kp_id;
p.Ki_iq = p.Ki_id;
p.Kaw_iq = p.Kaw_id;
p.VLimitRatio = single(0.577);

tau_current_cl = 1 / (2 * pi * 800);
tau_speed_delay = 1.5 * p.simcfg.Ts_speed;
tau_sigma = tau_current_cl + tau_speed_delay;
speed_bandwidth_hz = min(1 / (2 * pi * 3 * tau_sigma), 40);
speed_bandwidth_rad_s = 2 * pi * speed_bandwidth_hz;
p.Kp_speed = single(2 * speed_bandwidth_rad_s * J / torque_constant);
p.Ki_speed = single(speed_bandwidth_rad_s^2 * J / torque_constant);
p.Kaw_speed = single(40);
p.IqLimitDefault = single(p.inverter.current_limit);

p.openloop.vd = single(0);
p.openloop.vq = single(8);
p.openloop.freq_hz = single(20);
end

function close_open_data_dictionaries()
try
    Simulink.data.dictionary.closeAll('-discard');
catch err
    warning('Could not close open data dictionaries before rebuild: %s', ...
        err.message);
end
end

function dd = recreate_data_dictionary(cfg)
if exist(cfg.dictionaryFile, 'file')
    delete(cfg.dictionaryFile);
end

dd = Simulink.data.dictionary.create(cfg.dictionaryFile);
end

function upsert_alias_type(section, type_name, base_type, header_file)
alias_type = Simulink.AliasType(base_type);
alias_type.DataScope = 'Exported';
alias_type.HeaderFile = header_file;
upsert_entry(section, type_name, alias_type);
end

function upsert_bus_type(section, bus_name, header_file, element_specs)
bus_elements = repmat(Simulink.BusElement, numel(element_specs), 1);
for i = 1:numel(element_specs)
    bus_elements(i).Name = element_specs{i}{1};
    bus_elements(i).DataType = element_specs{i}{2};
    bus_elements(i).Dimensions = 1;
end

bus_type = Simulink.Bus;
bus_type.Elements = bus_elements;
bus_type.DataScope = 'Exported';
bus_type.HeaderFile = header_file;
upsert_entry(section, bus_name, bus_type);
end

function upsert_parameter(section, name, value, data_type)
parameter = Simulink.Parameter(double(value));
parameter.DataType = data_type;
parameter.CoderInfo.StorageClass = 'Auto';
upsert_entry(section, name, parameter);
end

function upsert_entry(section, name, value)
entry = find(section, 'Name', name);
if isempty(entry)
    addEntry(section, name, value);
else
    setValue(entry(1), value);
end
end
