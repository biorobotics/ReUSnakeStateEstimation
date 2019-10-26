%% - connect to snake. search the module on the bot
bot = HebiLookup.newGroupFromFamily('RUSNAKE');


numModules = bot.getNumModules;
ones_n = ones(1,numModules);
zeros_n = zeros(1,numModules);

% set low level controller strategy and gains
gains = bot.getGains();
gains.controlStrategy = ones_n*3;
gains.positionKp = ones_n*3.2; 
gains.positionKi = ones_n*0.031; 
gains.positionKd = ones_n*0.0; 

gains.velocityKp = ones_n*3;
gains.velocityKi = ones_n*0.2;
gains.velocityKd = ones_n*1;

gains.effortKi = ones_n*0.55;
gains.effortKd = ones_n*0.02;
bot.set('gains',gains);
pause(0.5);

% - start parameters
cmd = CommandStruct(); % initialize the command structure

% fbkHz = 200;
% bot.setFeedbackFrequencyHz(fbkHz); % set feedback rate, if desired

commandedAngles =  0*zeros(1, numModules);
prev_commandedAngles = 0*zeros(1, numModules);
commandedVels =    0*zeros(1, numModules);
commandedEfforts = 0*zeros(1, numModules);

%initial pos
cmd.position = commandedAngles;
cmd.velocity = nan(1,numModules);
cmd.effort = nan(1,numModules);

%% setup controller
joy = vrjoystick(1);  % must have use joystick attached to it
on_button = 0; % bot is on at startup, press shiney button on the joystick to exit control
% Set variables for buttons
a     = [1 0 0 0 0 0 0 0 0 0 0];  % buttons(1)
b     = [0 1 0 0 0 0 0 0 0 0 0];  % buttons(2)  
x     = [0 0 1 0 0 0 0 0 0 0 0];  
y     = [0 0 0 1 0 0 0 0 0 0 0];
lb     = [0 0 0 0 1 0 0 0 0 0 0];  % buttons(5)  
rb     = [0 0 0 0 0 1 0 0 0 0 0]; % buttons(6)  
select     = [0 0 0 0 0 0 1 0 0 0 0];  % buttons(7)  
option     = [0 0 0 0 0 0 0 1 0 0 0];   % buttons(8)  

%% setup UI
fk_fig = figure('Name','Snake FK');
fk_fig.Position = [100,100,800,600];
fk_axh = axes('Parent',fk_fig); % create axes
% need to be adjusted later
xbound = [-0.2;1.2];
ybound = [-0.5; 0.5];
zbound = [-0.5; 0.5];

axis([xbound(1) xbound(2) ybound(1) ybound(2) zbound(1) zbound(2)]);
grid on;
gca.DataAspectRatio = [1 1 1];
az = 45;
el = 30;
view(az, el);

% plot once 
g = snake_fk_cal(cmd.position);
fk_axh = snake_fk_plot(g,eye(4),fk_axh,1);

% 
% % test refresh
% for i = 1:1200
%     angle = [0.6*sin(i/150);0.6*cos(i/150+50);-0.6*cos(i/150+150);...
%              0.6*sin(i/150);0.6*cos(i/150+50);-0.6*cos(i/150+150);...
%              0.6*sin(i/150);0.6*cos(i/150+50);-0.6*cos(i/150+150)];
%     g = snake_fk_cal(angle);
%     snake_fk_plot(g,fk_axh,0);
%     pause(0.001)
% end
