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
        ChopperMode                 % 0 = PWM chopper, 1 = voltage chopper
        Acceleration                % acceleration (full steps / s^2)
        MaxSpeed                    % peak velocity (full steps / s)
        Position                    % absolute position
    end

    properties (Access = private)
        privMaxSpeed                % private: peak velocity
        privAcceleration            % private: acceleration
        privRMScurrent              % private: RMS current
        privChopper                 % private: Chopper mode
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
            obj.Port.write('GC', 'uint8');
            obj.privChopper = obj.Port.read(1, 'uint8');
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
            validateattributes(newCurrent,{'numeric'},...
                {'scalar','integer','nonnegative'})
            obj.Port.write('I', 'uint8', newCurrent, 'uint16', 'GI', 'uint8');
            obj.privRMScurrent = obj.Port.read(1, 'uint16');
        end

        function out = get.ChopperMode(obj)
            out = obj.privChopper;
        end
        function set.ChopperMode(obj, mode)
            validateattributes(mode,{'numeric'},...
                {'scalar','integer','nonnegative','<=',1})
            obj.Port.write('C', 'uint8', mode, 'uint8', 'GC', 'uint8');
            obj.privChopper = obj.Port.read(1, 'uint8');
        end

        function out = get.Position(obj)
            obj.Port.write('GP', 'uint8');
            out = obj.Port.read(1, 'int16');
        end
        function set.Position(obj,position)
            validateattributes(position,{'numeric'},{'scalar','integer'})
            obj.Port.write('P', 'int8', position, 'int16');
        end
        function resetPosition(obj)
            % Reset value of absolute position to zero.
            obj.Port.write('Z', 'uint8');
        end

        function step(obj, nSteps)
            % Move stepper motor a set number of steps. nSteps = positive
            % for clockwise steps, negative for counterclockwise
            validateattributes(nSteps,{'numeric'},{'scalar','integer'})
            obj.Port.write('S', 'uint8', nSteps, 'int16');
        end

        function setTarget(obj, id, target)
            validateattributes(id,{'numeric'},...
                {'scalar','integer','>=',1,'<=',9})
            validateattributes(target,{'numeric'},{'scalar','integer',...
                '>=',intmin('int32'),'<=',intmax('int32')})
            obj.Port.write(['T' id], 'uint8');
            obj.Port.write(target, 'int32');
        end

        function out = getTarget(obj, id)
            validateattributes(id,{'numeric'},...
                {'scalar','integer','>=',1,'<=',9})
            obj.Port.write(['G' id], 'uint8');
            out = obj.Port.read(1, 'int32');
        end

        function moveToTarget(obj,id)
            validateattributes(id,{'numeric'},...
                {'scalar','integer','nonnegative','>=',1,'<=',9})
            obj.Port.write(id, 'uint8');
        end

        function softStop(obj)
            obj.Port.write('x', 'uint8');
        end

        function hardStop(obj)
            obj.Port.write('X', 'uint8');
        end

        function rotate(obj,dir)
            % Turn stepper motor until limit switch is reached. Dir = 1
            % (clockwise, default) or -1 (counterclockwise)
            if ~exist('dir','var')
                dir = 1;
            else
                validateattributes(dir,{'numeric'},...
                    {'scalar','integer','nonzero','>=',-1,'<=',1})
            end
            if dir > 0
                obj.Port.write('F');
            else
                obj.Port.write('B');
            end
        end

        function varargout = getResistor(obj, idx)
            % Return the input resistor for IO port IDX
            %   0 = no input resistor
            %   1 = pullup resistor
            %   2 = pulldown resistor
            nargoutchk(0,1)
            narginchk(1,2)
            if ~exist('idx','var')
                idx = 1:6;
            else
                validateattributes(idx,{'numeric'},...
                    {'2d','increasing','integer','>=',1,'<=',6})
            end
            out = nan(1,numel(idx));
            for ii = 1:numel(idx)
                obj.Port.write(['GR' idx(ii)], 'uint8');
                out(ii) = obj.Port.read(1, 'uint8');
            end
            if nargout
                varargout{1} = out;
            else
                tmp = {'floating', 'pull-up', 'pull-down'};
                for ii = 1:numel(out)
                    fprintf('IO%d: %d (%s)\n',idx(ii),out(ii),tmp{out(ii)+1});
                end
            end
        end

        function setResistor(obj, idx, R)
            % Set the input resistor R for IO port IDX
            %   R = 0: no input resistor
            %   R = 1: pullup resistor
            %   R = 2: pulldown resistor
            validateattributes(idx,{'numeric'},...
                {'2d','increasing','integer','>=',1,'<=',6})
            validateattributes(R,{'numeric'},...
                {'scalar','integer','>=',0,'<=',2})
            for ii = 1:numel(idx)
                obj.Port.write(['R' idx(ii) R], 'uint8');
            end
        end

        function varargout = getMode(obj, idx)
            % Return the currently configured mode of IO port IDX
            nargoutchk(0,1)
            narginchk(1,2)
            if ~exist('idx','var')
                idx = 1:6;
            else
                validateattributes(idx,{'numeric'},...
                    {'2d','increasing','integer','>=',1,'<=',6})
            end
            out = nan(1,numel(idx));
            for ii = 1:numel(idx)
                obj.Port.write(['GM' idx(ii)], 'uint8');
                out(ii) = obj.Port.read(1, 'uint8');
            end
            if nargout
                varargout{1} = out;
            else
                for ii = 1:numel(out)
                    fprintf('IO%d: %d (''%c'')\n',idx(ii),out(ii),out(ii));
                end
            end
        end

        function setMode(obj, idx, M)
            % Set the input mode M for IO port IDX (cf. readme)
            validateattributes(idx,{'numeric'},...
                {'2d','increasing','integer','>=',1,'<=',6})
            validateattributes(M,{'numeric','char'},{'scalar'})
            for ii = 1:numel(idx)
                obj.Port.write(['M' idx(ii) M], 'uint8');
            end
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

        function liveView(obj)
            BpodStepperLive(obj);
        end
    end
end
