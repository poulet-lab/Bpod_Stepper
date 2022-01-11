classdef BpodStepperLive < handle

properties (Access = private)
    stepper
    dRec    = 10        % duration of data stored in ring-buffer [s]
    ts      = .05       % sample interval [s]
    lData               % length of ring-buffer
    data                % ring-buffer for storing data
    h
    t0      = nan       % value of first time-stamp
    fCLK;               % oscillatory frequency of driver clock [MHz]
end

methods
    function obj = BpodStepperLive(stepper)
        validateattributes(stepper,{'BpodStepperModule'},{'scalar'})
        obj.stepper = stepper;
        obj.stepper.Port.Port.BytesAvailableFcn = @obj.update;
        
        switch obj.stepper.DriverVersion
            case 2130
                obj.fCLK = 13.2E6;
            case 5160
                obj.fCLK = 12E6;
            otherwise
                error('Driver IC is not supported.')
        end
        
        obj.lData     = obj.dRec / obj.ts;
        obj.data      = nan(3,obj.lData);
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
        ylim([0 obj.stepper.MaxSpeed*1.2])
        set(gca,'XTickLabel',[])

        obj.h.axes(2) = subplot(3,1,2);
        obj.h.plot(2) = plot(obj.h.axes(2),NaN,NaN,'k','linewidth',2);
        title('acceleration')
        ylabel('|a| [steps/s^2]')
        ylim([0 obj.stepper.Acceleration*1.2])
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
            'ygrid',    'on', ...
            'box',      'off')
        linkaxes(obj.h.axes,'x')
        drawnow
        
        flushinput(obj.stepper.Port.Port)
        obj.stepper.Port.write(['L' 1], 'uint8');
    end

    function update(obj,~,~)
        incoming = double(obj.stepper.Port.read(3, 'uint32'));
        obj.data = [obj.data(:,2:obj.lData) incoming];
        
        if isnan(obj.t0)
            obj.t0 = incoming(1) / 1E3;
        end
        
        t = obj.data(1,:)/1E3-obj.t0;
        v = obj.tstep2step(obj.data(3,:));
        
        [obj.h.plot.XData]  = deal(t);
        obj.h.plot(1).YData = v;
        obj.h.plot(2).YData = [abs(diff(v)) NaN] / obj.ts;
        obj.h.plot(3).YData = bitand(1023,obj.data(2,:));

        xlim(obj.h.axes(1), min(t) + [0 obj.dRec])
        drawnow limitrate
    end

    function out = tstep2step(obj,in)
        persistent fact;
        if isempty(fact)
            fact = obj.fCLK / 256;
        end
        out = fact ./ in;
    end
    
    function delete(obj,~,~)
        obj.stepper.Port.write(['L' 0], 'uint8');
        obj.stepper.Port.Port.BytesAvailableFcn = '';
        flushinput(obj.stepper.Port.Port)
    end
end

end
