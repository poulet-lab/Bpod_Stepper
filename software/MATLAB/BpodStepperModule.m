%{
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, version 3.

This program is distributed  WITHOUT ANY WARRANTY and without even the
implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
%}

classdef BpodStepperModule < handle & matlab.mixin.CustomDisplay

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
        MicroPosition
        Position                    % absolute position
        EncoderPosition
    end

    properties (Dependent, Access = {?BpodStepperLive})
        StreamingMode
    end

    properties (Access = private)
        privUseEncoder = false
        privMaxSpeed                % private: peak velocity
        privAcceleration            % private: acceleration
        privRMScurrent              % private: RMS current
        privChopper                 % private: Chopper mode
        privStreamingMode = false   % private: streaming mode
        CurrentFirmwareVersion = 2  % most recent firmware version
    end

    methods
        function obj = BpodStepperModule(portString)

            % handle empty portString
            if ~exist('portString','var')
                portString = [];
            end

            % connect to stepper module
            obj.Port = ArCOMObject(portString, 115200, 'java', ...
                'BytesAvailableFcnCount', 12);
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
                case 17  % 0x11
                    obj.DriverVersion = 2130;
                case 48  % 0x30
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
            obj.privUseEncoder = isequal(obj.getMode(1:2),'ab');
        end

        function out = get.MaxSpeed(obj)
            out = obj.privMaxSpeed;
        end
        function set.MaxSpeed(obj, newSpeed)
            obj.Port.write('V', 'uint8', newSpeed, 'uint16');
            obj.pauseStreaming(true);
            obj.Port.write('GV', 'uint8');
            obj.privMaxSpeed = obj.Port.read(1, 'uint16');
            obj.pauseStreaming(false);
        end

        function out = get.Acceleration(obj)
            out = obj.privAcceleration;
        end
        function set.Acceleration(obj, newAccel)
            obj.Port.write('A', 'uint8', newAccel, 'uint16');
            obj.pauseStreaming(true);
            obj.Port.write('GA', 'uint8');
            obj.privAcceleration = obj.Port.read(1, 'uint16');
            obj.pauseStreaming(false);
        end

        function out = get.RMScurrent(obj)
            out = obj.privRMScurrent;
        end
        function set.RMScurrent(obj, newCurrent)
            validateattributes(newCurrent,{'numeric'},...
                {'scalar','integer','nonnegative'})
            obj.Port.write('I', 'uint8', newCurrent, 'uint16');
            obj.pauseStreaming(true);
            obj.Port.write('GI', 'uint8');
            obj.privRMScurrent = obj.Port.read(1, 'uint16');
            obj.pauseStreaming(false);
        end

        function out = get.ChopperMode(obj)
            out = obj.privChopper;
        end
        function set.ChopperMode(obj, mode)
            validateattributes(mode,{'numeric'},...
                {'scalar','integer','nonnegative','<=',1})
            obj.Port.write('C', 'uint8', mode, 'uint8');
            obj.pauseStreaming(true);
            obj.Port.write('GC', 'uint8');
            obj.privChopper = obj.Port.read(1, 'uint8');
            obj.pauseStreaming(false);
        end

        function out = get.MicroPosition(obj)
            obj.pauseStreaming(true);
            obj.Port.write('Gp', 'uint8');
            out = obj.Port.read(1, 'int32');
            obj.pauseStreaming(false);
        end
        function set.MicroPosition(obj,position)
            validateattributes(position,{'numeric'},{'scalar','integer'})
            obj.Port.write('p', 'int8', position, 'int32');
        end
        function out = get.Position(obj)
            obj.pauseStreaming(true);
            obj.Port.write('GP', 'uint8');
            out = obj.Port.read(1, 'int16');
            obj.pauseStreaming(false);
        end
        function set.Position(obj,position)
            validateattributes(position,{'numeric'},{'scalar','integer'})
            obj.Port.write('P', 'int8', position, 'int16');
        end
        function resetPosition(obj)
            % Reset value of absolute position to zero.
            obj.Port.write('Z', 'uint8');
        end

        function out = get.EncoderPosition(obj)
            obj.pauseStreaming(true);
            obj.Port.write('GN', 'uint8');
            out = obj.Port.read(1, 'int32');
            obj.pauseStreaming(false);
        end
        function resetEncoderPosition(obj)
            obj.Port.write('z', 'uint8');
        end

        function step(obj, nSteps)
            % Move stepper motor a set number of steps. nSteps = positive
            % for clockwise steps, negative for counterclockwise
            validateattributes(nSteps,{'numeric'},{'scalar','integer'})
            obj.Port.write('S', 'uint8', nSteps, 'int16');
        end
        
        function microStep(obj, nSteps)
            % Move stepper motor a set number of micro-steps. nSteps = positive
            % for clockwise steps, negative for counterclockwise
            validateattributes(nSteps,{'numeric'},{'scalar','integer'})
            obj.Port.write('s', 'uint8', nSteps, 'int32');
        end

        function setTarget(obj, varargin)
            nargoutchk(0,0)
            narginchk(3,5)
            varargin(nargin:4) = {[]};

            if isempty(varargin{1})
                varargin{1} = 1:9;
            else
                validateattributes(varargin{1},{'numeric'},{'2d',...
                    'increasing','integer','>=',1,'<=',9},mfilename,'ID')
            end

            state = cell(1,3);
            if any(cellfun(@isempty,varargin(2:end)))
                [state{:}] = obj.getTarget(varargin{1});
            end
            for ii = 2:4
                if isempty(varargin{ii})
                    varargin{ii} = state{ii-1};
                elseif isscalar(varargin{ii})
                    varargin{ii} = repmat(varargin{ii},size(varargin{1}));
                end
                varargin{ii}(isnan(varargin{ii})) = 0;
            end
            [id, p, a, v] = varargin{:};

            s = size(id);
            validateattributes(p,{'numeric'},{'integer','size',s,...
                '>=',intmin('int32'),'<=',intmax('int32')},'','Position')
            validateattributes(a,{'numeric'},{'integer','size',s,...
                'nonnegative','<=',intmax('uint16')},'','Acceleration')
            validateattributes(v,{'numeric'},{'integer','size',s,...
                'nonnegative','<=',intmax('uint16')},'','Peak Acceleration')

            for ii = 1:numel(id)
                obj.Port.write(['T' id(ii)],'uint8',p(ii),'int32',...
                    [a(ii) v(ii)],'uint16')
            end
        end

        function varargout = getTarget(obj, id)
            nargoutchk(0,3)
            narginchk(1,2)
            if ~exist('id','var')
                id = 1:9;
            else
                validateattributes(id,{'numeric'},...
                    {'2d','increasing','integer','>=',1,'<=',9})
            end
            n   = numel(id);
            out = zeros(n,3);
            obj.pauseStreaming(true);
            for ii = 1:n
                obj.Port.write(['G' id(ii)], 'uint8');
                out(ii,1)   = obj.Port.read(1, 'int32');
                out(ii,2:3) = obj.Port.read(2, 'uint16');
            end
            obj.pauseStreaming(false);
            out([false(n,1) out(:,2:3)==0]) = NaN;
            if nargout
                varargout = mat2cell(out',[1 1 1],n);
            else
                rn = arrayfun(@(x) {sprintf('Target #%d',x)},id);
                t = table(out(:,1),out(:,2),out(:,3),'VariableNames',...
                    {'Position', 'Acceleration', 'Peak Velocity'}, ...
                    'RowNames',rn);
                disp(t)
            end
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
            out = zeros(1,numel(idx));
            obj.pauseStreaming(true);
            for ii = 1:numel(idx)
                obj.Port.write(['GR' idx(ii)], 'uint8');
                out(ii) = obj.Port.read(1, 'uint8');
            end
            obj.pauseStreaming(false);
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

        function varargout = getMode(obj, id)
            % Return the currently configured mode of IO port IDX
            nargoutchk(0,1)
            narginchk(1,2)
            if ~exist('id','var')
                id = 1:6;
            else
                validateattributes(id,{'numeric'},...
                    {'2d','increasing','integer','>=',1,'<=',6},'','ID')
            end
            out = zeros(1,numel(id));
            obj.pauseStreaming(true);
            for ii = 1:numel(id)
                obj.Port.write(['GM' id(ii)], 'uint8');
                out(ii) = obj.Port.read(1, 'uint8');
            end
            obj.pauseStreaming(false);
            if nargout
                varargout{1} = out;
            else
                for ii = 1:numel(out)
                    fprintf('IO%d: %3d ',id(ii),out(ii));
                    if out(ii) == 0
                        fprintf('(unconfigured)\n');
                    elseif out(ii) < 10
                        fprintf('(go to predefined target #%d)\n',out(ii));
                    else
                        switch out(ii)
                            case 'a'
                                detail = ' - incremental encoder signal A';
                            case 'b'
                                detail = ' - incremental encoder signal B';
                            otherwise
                                detail = '';
                        end
                        fprintf('(''%c''%s)\n',out(ii),detail);
                    end
                end
            end
        end

        function setMode(obj, id, M)
            % Set the input mode M for IO port ID (cf. readme)
            validateattributes(id,{'numeric'},...
                {'2d','increasing','integer','>=',1,'<=',6},'','ID')
            if isscalar(M)
                M = repmat(M,size(id));
            end
            validateattributes(M,{'numeric','char'},{'size',size(id)})
            for ii = 1:numel(id)
                obj.Port.write(['M' id(ii) M(ii)], 'uint8');
            end
            obj.privUseEncoder = isequal(obj.getMode(1:2),'ab');
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

        function out = get.StreamingMode(obj)
            out = obj.privStreamingMode;
        end

        function set.StreamingMode(obj,enable)
            if xor(obj.privStreamingMode,enable)
                if enable
                    obj.Port.flush
                    obj.Port.write(['L' 1], 'uint8');
                else
                    obj.Port.write(['L' 0], 'uint8');
                    obj.Port.flush
                end
                obj.privStreamingMode = enable;
            end
        end
    end

    methods (Access = private)
        function pauseStreaming(obj,doPause)
            persistent BytesAvailableFcn isPaused;
            if isempty(BytesAvailableFcn)
                BytesAvailableFcn = '';
                isPaused = false;
            end
            if doPause
                if isPaused || ~obj.privStreamingMode
                    return
                end
                BytesAvailableFcn = obj.Port.BytesAvailableFcn;
                obj.Port.BytesAvailableFcn = '';
                obj.StreamingMode = false;
                isPaused = true;
            else
                if ~isPaused || obj.privStreamingMode
                    return
                end
                obj.Port.BytesAvailableFcn = BytesAvailableFcn;
                obj.StreamingMode = true;
                isPaused = false;
            end
        end
    end

    methods (Access = protected)
        function propgrp = getPropertyGroups(obj)
            gTitle1 = '<strong>Module Information</strong>';
            propList1 = {'HardwareVersion','DriverVersion','FirmwareVersion'};
            gTitle2 = '<strong>Motor Configuration</strong>';
            propList2 = {'RMScurrent','ChopperMode','MaxSpeed','Acceleration'};
            gTitle3 = '<strong>Movement Parameters</strong>';
            if obj.privUseEncoder
                propList3 = {'Position','EncoderPosition'};
            else
                propList3 = {'Position'};
            end
            propgrp(1) = matlab.mixin.util.PropertyGroup(propList1,gTitle1);
            propgrp(2) = matlab.mixin.util.PropertyGroup(propList2,gTitle2);
            propgrp(3) = matlab.mixin.util.PropertyGroup(propList3,gTitle3);
        end

        function s = getFooter(obj)
            s = matlab.mixin.CustomDisplay.getDetailedFooter(obj);
        end
    end
end
