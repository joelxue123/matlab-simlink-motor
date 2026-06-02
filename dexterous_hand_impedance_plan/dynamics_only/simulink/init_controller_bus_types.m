function buses = init_controller_bus_types()
%INIT_CONTROLLER_BUS_TYPES Define bus objects for struct-style controller I/O.
% The bus objects are assigned into the base workspace for Simulink.

input_bus = Simulink.Bus;
input_bus.Description = 'Controller input struct for usr_pid';
input_bus.Elements = [
    make_element('q_ref', 'fixdt(1,16,12)')
    make_element('q', 'fixdt(1,16,12)')
    make_element('qdot', 'fixdt(1,16,8)')
    make_element('qddot', 'fixdt(1,16,4)')
    make_element('tau_prev', 'fixdt(1,16,12)')
    make_element('mode', 'uint8')
];

output_bus = Simulink.Bus;
output_bus.Description = 'Controller output struct for usr_pid';
output_bus.Elements = [
    make_element('tau_cmd', 'fixdt(1,16,13)')
    make_element('tau_load_hat', 'fixdt(1,16,13)')
];

assignin('base', 'ControllerInputBus', input_bus);
assignin('base', 'ControllerOutputBus', output_bus);

buses = struct();
buses.ControllerInputBus = input_bus;
buses.ControllerOutputBus = output_bus;
end

function element = make_element(name, data_type)
    element = Simulink.BusElement;
    element.Name = name;
    element.DataType = data_type;
    element.Dimensions = 1;
    element.SampleTime = -1;
    element.Complexity = 'real';
end
