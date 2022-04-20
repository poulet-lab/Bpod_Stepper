classdef BpodStepperLive < handle

properties (Access = private)
    s
    dRec    = 10        % duration of data stored in ring-buffer [s]
    ts      = .05       % sample interval [s]
    data                % ring-buffer for storing data
    h                   % handles
    vDiv                % dividend for calculating steps/s from TSTEP
    yMax
    yNow    = inf(3,1)
end

methods
    function obj = BpodStepperLive(stepper)
        validateattributes(stepper,{'BpodStepperModule'},{'scalar'})
        obj.s = stepper;
        obj.s.Port.BytesAvailableFcn = @obj.update;

        switch obj.s.DriverVersion
            case 2130
                obj.vDiv = 13.2E6 / 256;
            case 5160
                obj.vDiv = 12.0E6 / 256;
            otherwise
                error('Driver IC is not supported.')
        end

        obj.dRec      = obj.dRec + 10*obj.ts;
        obj.data      = nan(4,obj.dRec/obj.ts);
        obj.data(1,:) = (-obj.dRec/obj.ts:-1)*obj.ts;

        obj.h.figure = figure('Visible','off');
        obj.yMax = [obj.s.MaxSpeed; obj.s.Acceleration; 10];

        t = {'Velocity','Acceleration','Mechanical Load'};
        for ii = 1:3
            obj.h.axes(ii) = subplot(3,1,ii);
            obj.h.plot(ii) = line(obj.h.axes(ii),obj.data(1,:),...
                obj.data(ii,:),'Color','k','linewidth',2);
            title(obj.h.axes(ii),t{ii})
        end
        xlim(obj.h.axes, [-obj.dRec+10*obj.ts 0])
        ylim(obj.h.axes, [0 inf])
        ylabel(obj.h.axes(1),'v [steps/s]')
        ylabel(obj.h.axes(2),'|a| [steps/s^2]')
        xlabel(obj.h.axes(end),'time [s]')
        set(obj.h.axes(3),'YDir','reverse')
        set(obj.h.axes, ...
            'TickDir',      'out', ...
            'XGrid',        'on', ...
            'YGrid',        'on', ...
            'XTickLabel',   [])
        linkaxes(obj.h.axes,'x')

        set(obj.h.figure, ......
            'DeleteFcn',        @obj.delete, ...
            'ToolBar',          'none', ...
            'MenuBar',          'none', ...
            'DockControls',     'off', ...
            'NumberTitle',      'off', ...
            'HandleVisibility', 'off', ...
            'Name',             'Stepper Motor Module — Live View')
        movegui(obj.h.figure,'center')
        drawnow
        set(obj.h.figure,'Visible','on')

        obj.s.StreamingMode = true;
    end

    function update(obj,~,~)
        incoming = double(obj.s.Port.read(3, 'uint32'));

        if isnan(obj.data(1,end))
           obj.data(1,:) = obj.data(1,:) + incoming(1)/1E3;
        end

        tmp     = nan(4,1);
        tmp(1)  = incoming(1) / 1E3;                        % time-stamp
        tmp(2)  = obj.vDiv ./ incoming(3);                  % velocity
        tmp(3)  = abs(tmp(2)-obj.data(2,end)) / obj.ts;     % acceleration
        tmp(4)  = bitand(1023, incoming(2));                % load

        if incoming(1) / 1E3 > obj.data(1,end) + 1.5 * obj.ts
            tmp(3)   = NaN;
            obj.data = [obj.data(:,3:end) nan(4,1) tmp];
        else
            obj.data = [obj.data(:,2:end) tmp];
        end

        [obj.h.plot.XData]  = deal(obj.data(1,:)-obj.data(1,end));
        obj.h.plot(1).YData = obj.data(2,:);
        obj.h.plot(2).YData = obj.data(3,:);
        obj.h.plot(3).YData = obj.data(4,:);

        dmax = max(obj.data(2:end,:),[],2) > obj.yMax;
        yinf = isinf(obj.yNow);
        for ii = 1:3
            if ~yinf(ii) && dmax(ii)
                obj.yNow(ii) = inf;
                obj.h.axes(ii).YLim(2) = obj.yNow(ii);
            elseif yinf(ii) && ~dmax(ii)
                obj.yNow(ii) = obj.yMax(ii);
                obj.h.axes(ii).YLim(2) = obj.yNow(ii);
            end
        end
        drawnow limitrate
    end

    function delete(obj,~,~)
        obj.s.Port.BytesAvailableFcn = '';
        obj.s.StreamingMode = false;
    end
end

end
