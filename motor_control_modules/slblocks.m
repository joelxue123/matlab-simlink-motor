function blkStruct = slblocks
%SLBLOCKS Register Motor Control Modules in the Simulink Library Browser.

Browser.Library = 'motor_control_lib';
Browser.Name = 'Motor Control Modules';
Browser.IsFlat = 0;

blkStruct.Browser = Browser;
end
