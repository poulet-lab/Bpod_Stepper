classdef BpodStepperLive < handle

properties (Access = private)
    s
    dRec    = 10        % duration of data stored in ring-buffer [s]
    ts      = .05       % sample interval [s]
    data                % ring-buffer for storing data
    h
    t0      = nan       % value of first time-stamp
    vDiv;               % dividend for calculating steps/s from TSTEP
    yMax
end

methods
    function obj = BpodStepperLive(stepper)
        validateattributes(stepper,{'BpodStepperModule'},{'scalar'})
        obj.s = stepper;
        obj.s.Port.Port.BytesAvailableFcn = @obj.update;

        switch obj.s.DriverVersion
            case 2130
                obj.vDiv = 13.2E6 / 256;
            case 5160
                obj.vDiv = 12.0E6 / 256;
            otherwise
                error('Driver IC is not supported.')
        end

        obj.data      = nan(3,obj.dRec/obj.ts);
        obj.data(2,:) = 0;
        
        obj.h.figure = figure('Visible','off');
        obj.yMax = [obj.s.MaxSpeed obj.s.Acceleration 10];
        for ii = 1:3
            obj.h.axes(ii) = subplot(3,1,ii);
            obj.h.plot(ii) = plot(obj.h.axes(ii),NaN,NaN,'k','linewidth',2);
            ylim(obj.h.axes(ii),[0 obj.yMax(ii)]);
        end
        title(obj.h.axes(1),'Velocity')
        title(obj.h.axes(2),'Acceleration')
        title(obj.h.axes(3),'Mechanical Load')
        ylabel(obj.h.axes(1),'v [steps/s]')
        ylabel(obj.h.axes(2),'|a| [steps/s^2]')
        xlabel(obj.h.axes(end),'time [s]')
        set(obj.h.axes(3),'YDir','reverse')
        set(obj.h.axes, ...
            'TickDir',  'out', ...
            'XGrid',    'on', ...
            'YGrid',    'on', ...
            'Box',      'off')
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

        flushinput(obj.s.Port.Port)
        obj.s.Port.write(['L' 1], 'uint8');
    end

    function update(obj,~,~)
        incoming = double(obj.s.Port.read(3, 'uint32'));
        obj.data = [obj.data(:,2:end) incoming];

        if isnan(obj.t0)
            obj.t0 = incoming(1) / 1E3;
        end
        t = obj.data(1,:) / 1E3 - obj.t0;
        v = obj.vDiv ./ obj.data(3,:);

        [obj.h.plot.XData]  = deal(t);
        obj.h.plot(1).YData = v;
        obj.h.plot(2).YData = [abs(diff(v)) NaN] / obj.ts;
        obj.h.plot(3).YData = bitand(1023, obj.data(2,:));

        for ii = 1:3
            tmp = max(obj.h.plot(ii).YData);
            if tmp > obj.yMax(ii)
                obj.yMax(ii) = ceil(tmp/10)*10;
                ylim(obj.h.axes(ii),[0 obj.yMax(ii)]);
            end
        end
        xlim(obj.h.axes(1), min(t) + [0 obj.dRec])
        drawnow limitrate
    end

    function delete(obj,~,~)
        obj.s.Port.write(['L' 0], 'uint8');
        obj.s.Port.Port.BytesAvailableFcn = '';
        flushinput(obj.s.Port.Port)
    end
end

end
