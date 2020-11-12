%{
----------------------------------------------------------------------------

This file is part of the Sanworks fork of the Poulet Lab Bpod_Stepper
repository: https://github.com/poulet-lab/Bpod_Stepper
Copyright (C) 2020 Sanworks LLC, Rochester, New York, USA

----------------------------------------------------------------------------

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, version 3.

This program is distributed  WITHOUT ANY WARRANTY and without even the
implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
%}

classdef BpodStepperModule < handle
    properties
        Port % ArCOM Serial port
        MaxSpeed
        Acceleration
    end
    properties (SetAccess = protected)
        FirmwareVersion = 0;
    end
    properties (Access = private)
        CurrentFirmwareVersion = 1;
        Initialized = 0; % Set to 1 after constructor finishes running
    end
    methods
        function obj = BpodStepperModule(portString)
            obj.Port = ArCOMObject_Stepper(portString, 115200);
            obj.Port.write(212, 'uint8');
            response = obj.Port.read(1, 'uint8');
            if response ~= 211
                error('Could not connect =( ')
            end
            obj.FirmwareVersion = obj.Port.read(1, 'uint32');
            if obj.FirmwareVersion < obj.CurrentFirmwareVersion
                error(['Error: old firmware detected - v' obj.FirmwareVersion '. The current version is: ' obj.CurrentFirmwareVersion '. Please update the Stepper Module firmware using Arduino.'])
            end
            obj.Initialized = 1;
            obj.Port.write('GA', 'uint8');
            obj.MaxSpeed = obj.Port.read(1, 'int16');
            obj.Port.write('GV', 'uint8');
            obj.Acceleration = obj.Port.read(1, 'int16');
        end
        function set.MaxSpeed(obj, newSpeed)
            if obj.Initialized
                obj.Port.write('V', 'uint8', newSpeed, 'int16');
            end
            obj.MaxSpeed = newSpeed;
        end
        function set.Acceleration(obj, newAccel)
            if obj.Initialized
                obj.Port.write('A', 'uint8', newAccel, 'int16');
            end
            obj.Acceleration = newAccel;
        end
        function step(obj, nSteps) % Move stepper motor a set number of steps. nSteps = positive for clockwise steps, negative for counterclockwise
            obj.Port.write('S', 'uint8', nSteps, 'int16');
        end
        function turnDegrees(obj, nDegrees) % Move stepper motor a set number of degrees. nDegrees = positive for clockwise, negative for counterclockwise
            obj.Port.write('D', 'uint8', nDegrees, 'int16');
        end
        function findLimitSwitch(obj, Dir) % Turn stepper motor until limit switch is reached. Dir = 0 (clockwise) or 1 (counterclockwise)
            obj.Port.write(['L' Dir], 'uint8');
        end
        function delete(obj)
            obj.Port = []; % Trigger the ArCOM port's destructor function (closes and releases port)
        end
    end
end
