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
    
    properties (SetAccess = protected)
        Port                        % ArCOM Serial port
        FirmwareVersion             % firmware version of connected module
        HardwareVersion             % PCB revision of connected module
        DriverVersion               % which TMC driver is installed?
    end
    
    properties (Dependent)
        RMScurrent                  % RMS current (mA)
        Acceleration                % acceleration (full steps / s^2)
        MaxSpeed                    % peak velocity (full steps / s)
        Position                    % absolute position
    end
    
    properties (Access = private)
        privMaxSpeed                % private: peak velocity
        privAcceleration            % private: acceleration
        privRMScurrent              % private: RMS current
        CurrentFirmwareVersion = 2  % most recent firmware version
    end
    
    methods
        function obj = BpodStepperModule(portString)

            % connect to stepper module
            obj.Port = ArCOMObject_Stepper(portString, 115200);
            obj.Port.write(212, 'uint8');
            if obj.Port.read(1, 'uint8') ~= 211
                error('Could not connect =( ')
            end
            
            % check firmware version
            obj.FirmwareVersion = obj.Port.read(1, 'uint32');
            if obj.FirmwareVersion < obj.CurrentFirmwareVersion
                error(['Error: old firmware detected - v%d. The ' ...
                    'current version is: v%d. Please update the ' ...
                    'Stepper Module firmware using Arduino.'], ...
                    obj.FirmwareVersion, obj.CurrentFirmwareVersion)
            end
            
            % get non-dependent parameters from stepper module
            obj.Port.write('GH', 'uint8');
            obj.HardwareVersion = double(obj.Port.read(1, 'uint8')) / 10;
            obj.Port.write('GT', 'uint8');
            switch obj.Port.read(1, 'uint8')
                case 0x11
                    obj.DriverVersion = 2130;
                case 0x30
                    obj.DriverVersion = 5160;
                otherwise
                    obj.DriverVersion = NaN;
            end
            obj.Port.write('GA', 'uint8');
            obj.privAcceleration = obj.Port.read(1, 'uint16');
            obj.Port.write('GV', 'uint8');
            obj.privMaxSpeed = obj.Port.read(1, 'uint16');
            obj.Port.write('GI', 'uint8');
            obj.RMScurrent = obj.Port.read(1, 'uint16');
        end
        
        function out = get.MaxSpeed(obj)
            out = obj.privMaxSpeed;
        end
        function set.MaxSpeed(obj, newSpeed)
            obj.Port.write('V', 'uint8', newSpeed, 'uint16');
            obj.Port.write('GV', 'uint8');
            obj.privMaxSpeed = obj.Port.read(1, 'uint16');
        end
        
        function out = get.Acceleration(obj)
            out = obj.privAcceleration;
        end
        function set.Acceleration(obj, newAccel)
            obj.Port.write('A', 'uint8', newAccel, 'uint16');
            obj.Port.write('GA', 'uint8');
            obj.privAcceleration = obj.Port.read(1, 'uint16');
        end
        
        function out = get.RMScurrent(obj)
            out = obj.privRMScurrent;
        end
        function set.RMScurrent(obj, newCurrent)
            obj.Port.write('I', 'uint8', newCurrent, 'uint16');
            obj.Port.write('GI', 'uint8');
            obj.privRMScurrent = obj.Port.read(1, 'uint16');
        end

        function out = get.Position(obj)
            obj.Port.write('GP', 'uint8');
            out = obj.Port.read(1, 'int16');
        end
        function set.Position(obj,position)
            obj.Port.write('P', 'int8', position, 'int16');
        end
        function resetPosition(obj)
            % Reset value of absolute position to zero.
            obj.Port.write('Z', 'uint8');
        end

        function step(obj, nSteps)
            % Move stepper motor a set number of steps. nSteps = positive
            % for clockwise steps, negative for counterclockwise
            obj.Port.write('S', 'uint8', nSteps, 'int16');
        end
        
        function setTarget(obj, id, target)
            validateattributes(id,{'numeric'},...
                {'scalar','integer','nonnegative','<',10})
            validateattributes(target,{'numeric'},{'scalar','integer',...
                '>=',intmin('int32'),'<=',intmax('int32')})
            obj.Port.write(sprintf('T%d',id), 'uint8');
            obj.Port.write(target, 'int32');
        end

        function out = getTarget(obj, id)
            validateattributes(id,{'numeric'},...
                {'scalar','integer','nonnegative','<',10})
            obj.Port.write(sprintf('G%d',id), 'uint8');
            out = obj.Port.read(1, 'int32');
        end
        
        function moveToTarget(id)
            validateattributes(id,{'numeric'},...
                {'scalar','integer','nonnegative','<',10})
            obj.Port.write(id+48, 'uint8');
        end

        function findLimitSwitch(obj, Dir)
            % Turn stepper motor until limit switch is reached. Dir = 0
            % (clockwise) or 1 (counterclockwise)
            obj.Port.write(['L' Dir>0], 'uint8');
        end

        function storeDefaults(obj)
            % Store current settings for acceleration, peak velocity and
            % RMS current as default values
            obj.Port.write('E', 'uint8');
        end

        function delete(obj)
            % Trigger the ArCOM port's destructor function (closes and
            % releases port)
            obj.Port = [];
        end
    end
end
