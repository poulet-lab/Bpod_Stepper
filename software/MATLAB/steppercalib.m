clear all
close all

m = BpodStepperModule;
m.resetPosition;
m.resetEncoderPosition;

m.MaxSpeed = 1500;

nrevs = 100;
m.Position = nrevs * 200;

tmp = m.EncoderPosition;
pause(.1)
while tmp ~= m.EncoderPosition
    tmp = m.EncoderPosition;
    pause(.1)
end

m.EncoderPosition / nrevs