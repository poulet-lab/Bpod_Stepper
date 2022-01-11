classdef BpodStepperLive < handle

properties (Access = private)
    stepper
    dRec    = 10        % duration of data stored in ring-buffer [s]
    ts      = .05       % sample interval [s]
    data                % ring-buffer for storing data
    h
    t0      = nan       % value of first time-stamp
    vDiv;               % dividend for calculating steps/s from TSTEP
end

methods
    function obj = BpodStepperLive(stepper)
        validateattributes(stepper,{'BpodStepperModule'},{'scalar'})
        obj.stepper = stepper;
        obj.stepper.Port.Port.BytesAvailableFcn = @obj.update;

        switch obj.stepper.DriverVersion
            case 2130
                obj.vDiv = 13.2E6 / 256;
            case 5160
                obj.vDiv = 12.0E6 / 256;
            otherwise
                error('Driver IC is not supported.')
        end

        obj.data      = nan(3,obj.dRec/obj.ts);
        obj.data(2,:) = 2^20-1;

        obj.h.figure = figure('Visible','off');

        tmp(1) = subplot(3,1,1);
        title(tmp(1),'Velocity')
        ylabel(tmp(1),'v [steps/s]')
        ylim(tmp(1),[0 obj.stepper.MaxSpeed*1.2])

        tmp(2) = subplot(3,1,2);
        title(tmp(2),'Acceleration')
        ylabel(tmp(2),'|a| [steps/s^2]')
        ylim(tmp(2),[0 obj.stepper.Acceleration*1.2])

        tmp(3) = subplot(3,1,3);
        title(tmp(3),'Mechanical Load')
        ylim(tmp(3),[0 1023])
        set(gca,'YDir','reverse')

        set(tmp, ...
            'XTick',    [], ...
            'XColor',   'none', ...
            'TickDir',  'out')
        for ii = 1:numel(tmp)
            obj.h.axes(ii) = axes();
            obj.h.plot(ii) = plot(obj.h.axes(ii),NaN,NaN,'k','linewidth',2);
            linkprop([tmp(ii) obj.h.axes(ii)],{'Position','YLim','YDir'});
        end

        set(obj.h.axes, ...
            'TickDir',  'out', ...
            'XGrid',    'on', ...
            'YGrid',    'on', ...
            'Box',      'off', ...
            'Ycolor',   'none', ...
            'Color',    'none')
        xlabel(obj.h.axes(end),'time [s]')
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
        set(obj.h.figure,'Visible','on')
        drawnow

        flushinput(obj.stepper.Port.Port)
        obj.stepper.Port.write(['L' 1], 'uint8');
    end

    function update(obj,~,~)
        incoming = double(obj.stepper.Port.read(3, 'uint32'));
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

        xlim(obj.h.axes(1), min(t) + [0 obj.dRec])
        drawnow limitrate
    end

    function delete(obj,~,~)
        obj.stepper.Port.write(['L' 0], 'uint8');
        obj.stepper.Port.Port.BytesAvailableFcn = '';
        flushinput(obj.stepper.Port.Port)
    end
end

end
