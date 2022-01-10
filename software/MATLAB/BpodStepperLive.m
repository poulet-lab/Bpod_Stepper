classdef BpodStepperLive < handle

properties (Access = private)
    stepper
    ldata = 200
    raw
    data
    h
    t0 = NaN
    fCLK = 13.2E6;
end

methods
    function obj = BpodStepperLive(stepper)
        validateattributes(stepper,{'BpodStepperModule'},{'scalar'})

        obj.stepper = stepper;
        obj.stepper.Port.Port.BytesAvailableFcn = @obj.update;

        obj.data  = nan(3,obj.ldata);
        obj.data(2,:) = 2^20-1;

        obj.h.figure = figure(...
            'DeleteFcn',    @obj.delete, ...
            'ToolBar',      'none', ...
            'MenuBar',      'none', ...
            'Name',         'Stepper Motor Module — Live View', ...
            'NumberTitle',  'off');
        obj.h.axes(1) = subplot(3,1,1);
        obj.h.plot(1) = plot(obj.h.axes(1),NaN,NaN,'k','linewidth',2);
        title('velocity')
        ylabel('v [steps/s]')
        ylim([0 obj.stepper.MaxSpeed*1.1])
        set(gca,'XTickLabel',[])

        obj.h.axes(2) = subplot(3,1,2);
        obj.h.plot(2) = plot(obj.h.axes(2),NaN,NaN,'k','linewidth',2);
        title('acceleration')
        ylabel('|a| [steps/s2]')
        ylim([0 obj.stepper.Acceleration*1.1])
        set(gca,'XTickLabel',[])

        obj.h.axes(3) = subplot(3,1,3);
        obj.h.plot(3) = plot(obj.h.axes(3),NaN,NaN,'k','linewidth',2);
        title('load')
        xlabel('time [s]')
        ylim([0 1023])
        set(gca,'YDir','reverse')

        set(obj.h.axes, ...
            'tickdir',  'out', ...
            'xgrid',    'on', ...
            'ygrid',    'on')
        linkaxes(obj.h.axes,'x')

        obj.stepper.Port.write(['L' 1], 'uint8');
    end

    function update(obj,~,~)

        incoming = obj.stepper.Port.read(3, 'uint32');
        obj.data = [obj.data(:,2:obj.ldata) double(incoming)];

        if isnan(obj.t0)
            obj.t0 = obj.data(1,find(~isnan(obj.data(1,:)),1)) / 1E3;
        end

        t = obj.data(1,:) / 1E3 - obj.t0;
        v = obj.tstep2step(obj.data(3,:));
        a = [abs(diff(v)) NaN] * 20;
        l = bitand(1023,obj.data(2,:));

        obj.h.plot(1).XData = t;
        obj.h.plot(1).YData = v;

        obj.h.plot(2).XData = t;
        obj.h.plot(2).YData = a;

        obj.h.plot(3).XData = t;
        obj.h.plot(3).YData = l;

        xlim(obj.h.axes(1),[max([0 min(t)]) max([10 max(t)])])

        drawnow limitrate
    end

    function delete(obj,~,~)
        obj.stepper.Port.write(['L' 0], 'uint8');
        obj.stepper.Port.Port.BytesAvailableFcn = '';
        fread(obj.stepper.Port.Port,obj.stepper.Port.Port.bytesAvailable);
    end

    function out = tstep2step(obj,in)
        out = 1 ./ (in * 256 / obj.fCLK);
    end
end

end
