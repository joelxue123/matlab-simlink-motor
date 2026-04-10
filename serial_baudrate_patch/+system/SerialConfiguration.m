classdef SerialConfiguration < matlab.System
    % Patched SerialConfiguration — adds support for non-standard baud rates
    % on Linux by catching the validation error from Serial.p and falling
    % back to the closest standard rate. For USB CDC virtual COM ports the
    % baud rate value is ignored by hardware, so this is transparent.
    %
    % Place the parent directory (serial_baudrate_patch/) on the MATLAB path
    % BEFORE the toolbox path so this file shadows the built-in copy.
    %
    % Original: toolbox/shared/seriallib/serialblks/+system/SerialConfiguration.m
    % Copyright 2020-2023 The MathWorks, Inc.  (original)
    % Patch Copyright 2024 — MIT License

    % Public, non-tunable properties
    properties(Nontunable)
        % Communication port
        PortSel = '<Select a port...>'
        % Baud rate
        BaudRate = 9600;
        % Data bits
        DataBits = '8';
        % Parity
        Parity = 'none'
        % Stop bits
        StopBits = 1;
        % Byte order
        ByteOrder = 'little-endian';
        % Flow control
        FlowControl = 'none';
        % Timeout
        Timeout = 10;
        ComPort = '<Select a port...>';
    end

    %#codegen
    properties (Access = 'private')
        SerialObj
    end

    properties (Hidden)
        DataBitsSet = matlab.system.StringSet({'5', '6', '7', '8'});
        ParitySet = matlab.system.StringSet({'none', 'even', 'odd'});
        ByteOrderSet = matlab.system.StringSet({'big-endian', 'little-endian'});
        FlowControlSet = matlab.system.StringSet({'none', 'hardware'});
    end

    properties (Constant, Hidden)
        % Maximum baud rate that Linux termios supports via standard Bxxx
        MaxStandardBaud = 4000000;
    end

    methods
        %% Constructor
        function obj = SerialConfiguration(varargin)
            setProperties(obj, nargin, varargin{:})
        end

        %% Set functions to validate and set the property value.
        function set.Timeout(obj, value)
            validateattributes(value, {'numeric'}, ...
                {'real', 'nonempty', 'nonnan', 'positive', 'scalar'}, ...
                '', 'Timeout');
            obj.Timeout = value;
        end

        function set.BaudRate(obj, value)
            validateattributes(value,{'numeric'}, ...
                { 'scalar', 'nonnegative', 'real', 'nonnan', 'integer'}, ...
                '', 'Baud rate')
            obj.BaudRate = value;
        end

        function set.StopBits(obj, value)
            if ismember(real(str2double(obj.DataBits)), [6, 7, 8])
                if (~any(ismember(value,[1, 2])) || ~isscalar(value))
                    coder.internal.error('instrument:instrumentblks:invalidStopbitsDatabits1');
                end
            else
                if ismember(real(str2double(obj.DataBits)), 5)
                    if (~any(ismember(value,[1, 1.5])) || ~isscalar(value))
                        coder.internal.error('instrument:instrumentblks:invalidStopbitsDatabits2');
                    end
                end
            end
            obj.StopBits = value;
        end

    end

    methods(Access = protected)
        %% Define number of inputs and outputs to the system.
        function num = getNumOutputsImpl(~)
            num = 0;
        end

        function num = getNumInputsImpl(~)
            num = 0;
        end

        %% Algorithm implementation — PATCHED for non-standard baud rates
        function setupImpl(obj)
            if strcmpi(obj.PortSel, '<Select a port...>')
                coder.internal.error('instrument:instrumentblks:noPortsSelected');
            end

            portValue = obj.PortSel;
            coder.extrinsic('serialportlist');
            coder.extrinsic('slResolve');
            coder.extrinsic('gcb');
            coder.extrinsic('ismember');
            blockPath = coder.const(gcb);
            slList = coder.const(serialportlist);
            portInList = coder.const(~ismember(obj.PortSel, slList));
            if portInList
                [portValue, portStatus]= coder.const(@slResolve,obj.PortSel, blockPath);
                if ~portStatus
                    portValue = obj.PortSel;
                end
            end

            requestedBaud = obj.BaudRate;
            effectiveBaud = requestedBaud;

            % If baud rate exceeds Linux standard maximum, use a fallback
            % that the internal Serial class accepts. For USB CDC virtual
            % COM ports the baud rate is ignored by hardware anyway.
            % The LD_PRELOAD shim (libbaudrate_shim.so) handles setting
            % the actual rate at the native level via BOTHER/TCSETS2.
            if requestedBaud > obj.MaxStandardBaud
                effectiveBaud = obj.MaxStandardBaud;
                fprintf(['[SerialConfiguration patch] Baud rate %g exceeds ' ...
                    'Linux standard limit (%d).\n' ...
                    '  Using %d for MATLAB serial layer ' ...
                    '(LD_PRELOAD shim handles the real rate).\n'], ...
                    requestedBaud, obj.MaxStandardBaud, effectiveBaud);
            end

            try
                obj.SerialObj = matlabshared.seriallib.internal.Serial(portValue, ...
                    'IsSharingPort', true, ...
                    'IsSharingExistingTimeout', false, ...
                    'IsWriteOnly', true, ...
                    'BaudRate', effectiveBaud, ...
                    'DataBits', real(str2double(obj.DataBits)), ...
                    'Parity', obj.Parity, ...
                    'StopBits', obj.StopBits, ...
                    'ByteOrder', obj.ByteOrder, ...
                    'FlowControl', obj.FlowControl, ...
                    'Timeout', obj.Timeout);
            catch ME
                % If even the fallback rate fails, try a conservative rate
                if effectiveBaud > 921600
                    fprintf(['[SerialConfiguration patch] Rate %d also ' ...
                        'failed, trying 921600.\n'], effectiveBaud);
                    obj.SerialObj = matlabshared.seriallib.internal.Serial(portValue, ...
                        'IsSharingPort', true, ...
                        'IsSharingExistingTimeout', false, ...
                        'IsWriteOnly', true, ...
                        'BaudRate', 921600, ...
                        'DataBits', real(str2double(obj.DataBits)), ...
                        'Parity', obj.Parity, ...
                        'StopBits', obj.StopBits, ...
                        'ByteOrder', obj.ByteOrder, ...
                        'FlowControl', obj.FlowControl, ...
                        'Timeout', obj.Timeout);
                else
                    rethrow(ME);
                end
            end

            obj.SerialObj.connect;

            if ~obj.SerialObj.InitAccess
                coder.internal.error('instrument:instrumentblks:errorConfiguringPort');
            end
        end

        function releaseImpl(obj)
            disconnect(obj.SerialObj)
        end
    end
end
