function MotorMelody     
global BpodSystem

% First, lets connect to the Bpod Stepper Module via USB.
% Use the Bpod GUI to define the module's USB port.
BpodSystem.assertModule({'Stepper'}, 1);
m = BpodStepperModule(BpodSystem.ModuleUSB.Stepper1);

% setting up the motor ... try the different chopper modes
m.ChopperMode =   1;
m.RMScurrent  = 200;

% Acceleration phases should be as short as possible
m.Acceleration = intmax('uint16');

% Some note definitions
NOTE_GS3 =  208;
NOTE_A3  =  220;
NOTE_B3  =  247;
NOTE_C4  =  262;
NOTE_D4  =  294;
NOTE_E4  =  330;
NOTE_GS4 =  415;
NOTE_A4  =  440;
NOTE_B4  =  494;
NOTE_C5  =  523;
NOTE_D5  =  587;
NOTE_E5  =  659;
NOTE_F5  =  698;
NOTE_G5  =  784;
NOTE_A5  =  880;
NOTE_XX  =    0; % Pause

% the melody line
melody =     [NOTE_E5,  NOTE_B4,  NOTE_C5,  NOTE_D5,  NOTE_C5,  NOTE_B4,...
    NOTE_A4,  NOTE_A4,  NOTE_C5,  NOTE_E5,  NOTE_D5,  NOTE_C5,  NOTE_B4,...
    NOTE_B4,  NOTE_C5,  NOTE_D5,  NOTE_E5,  NOTE_C5,  NOTE_A4,  NOTE_A4,...
    NOTE_XX,  NOTE_D5,  NOTE_F5,  NOTE_A5,  NOTE_G5,  NOTE_F5,  NOTE_E5,...
    NOTE_C5,  NOTE_E5,  NOTE_D5,  NOTE_C5,  NOTE_B4,  NOTE_B4,  NOTE_C5,...
    NOTE_D5,  NOTE_E5,  NOTE_C5,  NOTE_A4,  NOTE_A4,  NOTE_XX,  NOTE_E4,...
    NOTE_C4,  NOTE_D4,  NOTE_B3,  NOTE_C4,  NOTE_A3,  NOTE_GS3, NOTE_B3,...
    NOTE_E4,  NOTE_C4,  NOTE_D4,  NOTE_B3,  NOTE_C4,  NOTE_E4,  NOTE_A4,...
    NOTE_A4,  NOTE_GS4];

% durations for each note and pause in the melody
dur = [1, .5, .5, 1, .5, .5, 1, .5, .5, 1, .5, .5, 1, .5, .5, 1, 1, 1, ...
    1, 1, 1, 1, .5, 1, .5, .5, 1.5, .5, 1, .5, .5, 1, .5, .5, 1, 1, 1, ...
    1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 2];

% adjust tempo of playback
dur = .4 * dur;


%% Main loop
for ii = 1:length(melody)

    % we want to go back and forth - because ... why not
    if mod(ii,2)
        dir = 'F';
    else
        dir = 'B';
    end
    
    % the speed of the motor corresponds directly to the pitch of its sound
    m.MaxSpeed = melody(ii);

    % definition of the state machine
    sma = NewStateMachine();
    sma = AddState(sma, 'Name', 'play', ...
        'Timer', dur(ii), ...
        'StateChangeConditions', {'Tup', 'stop'},...
        'OutputActions', {'Stepper1', dir});
    sma = AddState(sma, 'Name', 'stop', ...
        'Timer', 0, ...
        'StateChangeConditions', {'Tup', 'exit'},...
        'OutputActions', {'Stepper1', 'X'});
    
    SendStateMatrix(sma); % Send state machine to Bpod
    RunStateMatrix;       % Run the trial
    
    HandlePauseCondition;
    if BpodSystem.Status.BeingUsed == 0
        m.hardStop;
        delete(m)
        return
    end
end
RunProtocol('Stop')
delete(m)