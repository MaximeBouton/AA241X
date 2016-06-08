clear all
d = 10; % turn radius
pts_turn = 8;

% create waypoint vector:
% x is East, y is North
% u = uE, uN

turn1st = 0/d;

% lead in to start gate
wpt_x(1,1) = -120;
wpt_y(1,1) = 78.6017;
wpt_z(1,1) = 0;

wpt_x(1,end+1) = -123;
wpt_y(1,end+1) = 137.6017;
wpt_z(1,end+1) = 0;
x = wpt_x(end) - wpt_x(end-1);
y = wpt_y(end) - wpt_y(end-1);
u(1,:) = [x y]/norm([x y]);

wpt_x(1,end+1) = -120;
wpt_y(1,end+1) = 150.6017;
wpt_z(1,end+1) = 0;
x = wpt_x(end) - wpt_x(end-1);
y = wpt_y(end) - wpt_y(end-1);
u(end+1,:) = [x y]/norm([x y]);

wpt_x(1,end+1) = -115;
wpt_y(1,end+1) = 155.6017;
wpt_z(1,end+1) = 0;
x = wpt_x(end) - wpt_x(end-1);
y = wpt_y(end) - wpt_y(end-1);
u(end+1,:) = [x y]/norm([x y]);

wpt_x(1,end+1) = -109;
wpt_y(1,end+1) = 157;
wpt_z(1,end+1) = 0;
x = wpt_x(end) - wpt_x(end-1);
y = wpt_y(end) - wpt_y(end-1);
u(end+1,:) = [x y]/norm([x y]);

% start
wpt_x(1,end+1) = -91;
wpt_y(1,end+1) = 152.6017;
wpt_z(1,end+1) = 0;
x = wpt_x(end) - wpt_x(end-1);
y = wpt_y(end) - wpt_y(end-1);
u(end+1,:) = [x y]/norm([x y]);

ang = 2*pi/pts_turn;

% turn 1
i = 1;
while (840*pi/180-ang*(i-1)) > 0
    if i == 1
        fix = turn1st;
    else
        fix = 0;
    end
wpt_x(1,end+1) = 125 + cos(1.134665-ang*(i-1))*d + fix;
wpt_y(1,end+1) = 47 + sin(1.134665-ang*(i-1))*d + fix;
wpt_z(1,end+1) = 2.5*(i-1)/pts_turn;
x = wpt_x(end) - wpt_x(end-1);
y = wpt_y(end) - wpt_y(end-1);
u(end+1,:) = [x y]/norm([x y]);
i = i+1;
end
left1 = 840*pi/180 - ang*(i-1); % leftover angle from 840
wpt_x(1,end+1) =  125 + cos(1.134665-ang*(i-1)-left1)*d;
wpt_y(1,end+1) =  47 + sin(1.134665-ang*(i-1)-left1)*d;
wpt_z(1,end+1) = wpt_z(1,end);
x = wpt_x(end) - wpt_x(end-1);
y = wpt_y(end) - wpt_y(end-1);
u(end+1,:) = [x y]/sqrt(x^2+y^2); % alternative to using norm()

% turn 2
i = 1;
while (840*pi/180-ang*(i-1)) > 0
    if i == 1
        fix = turn1st;
    else
        fix = 0;
    end
wpt_x(1,end+1) = -75 + cos(-0.96007-ang*(i-1))*d + fix;
wpt_y(1,end+1) = -93 + sin(-0.96007-ang*(i-1))*d - fix;
if i == 1
    wpt_z(1,end+1) = -1;
else
    wpt_z(1,end+1) = wpt_z(1,end) + 2.5/pts_turn;
end
x = wpt_x(end) - wpt_x(end-1);
y = wpt_y(end) - wpt_y(end-1);
u(end+1,:) = [x y]/norm([x y]);
i = i+1;
end

left2 = 840*pi/180 - ang*(i-1);
wpt_x(1,end+1) =  -75 + cos(-0.96007-ang*(i-1)-left2)*d;
wpt_y(1,end+1) =  -93 + sin(-0.96007-ang*(i-1)-left2)*d;
wpt_z(1,end+1) =  wpt_z(1,end);
x = wpt_x(end) - wpt_x(end-1);
y = wpt_y(end) - wpt_y(end-1);
u(end+1,:) = [x y]/norm([x y]);

% finish
wpt_x(1,end+1) = -104;
wpt_y(1,end+1) = 144.3983;
wpt_z(1,end+1) = -1;
x = wpt_x(end) - wpt_x(end-1);
y = wpt_y(end) - wpt_y(end-1);
u(end+1,:) = [x y]/norm([x y]);

% end smoothly
wpt_x(1,end+1) = -90;
wpt_y(1,end+1) = 155.3983;
wpt_z(1,end+1) = -1;
x = wpt_x(end) - wpt_x(end-1);
y = wpt_y(end) - wpt_y(end-1);
u(end+1,:) = [x y]/norm([x y]);

wpt_x(1,end+1) = -70;
wpt_y(1,end+1) = 155;
wpt_z(1,end+1) = -1;
x = wpt_x(end) - wpt_x(end-1);
y = wpt_y(end) - wpt_y(end-1);
u(end+1,:) = [x y]/norm([x y]);

wpt_x(1,end+1) = -60;
wpt_y(1,end+1) = 145;
wpt_z(1,end+1) = -1;
x = wpt_x(end) - wpt_x(end-1);
y = wpt_y(end) - wpt_y(end-1);
u(end+1,:) = [x y]/norm([x y]);

wpt_x(1,end+1) = -80;
wpt_y(1,end+1) = 85;
wpt_z(1,end+1) = -1;
x = wpt_x(end) - wpt_x(end-1);
y = wpt_y(end) - wpt_y(end-1);
u(end+1,:) = [x y]/norm([x y]);

wpt_x(1,end+1) = -95;
wpt_y(1,end+1) = 65;
wpt_z(1,end+1) = -1;
x = wpt_x(end) - wpt_x(end-1);
y = wpt_y(end) - wpt_y(end-1);
u(end+1,:) = [x y]/norm([x y]);

wpt_x(1,end+1) = -105;
wpt_y(1,end+1) = 70;
wpt_z(1,end+1) = -1;
x = wpt_x(end) - wpt_x(end-1);
y = wpt_y(end) - wpt_y(end-1);
u(end+1,:) = [x y]/norm([x y]);

% all together now (for input into control code)
u = [u; 0 0];
ENDuEuN = [wpt_x' wpt_y' wpt_z' u];


j = 1:1:length(wpt_x);
figure
plot(j,wpt_x,j,wpt_y)

% plotting
% close all
figure(1)
plot([125 -75],[47 -93],'b*');hold on
% plot([-96 125 -75 -96],[150 47 -93 150],'b--');
% plot([wpt_x],[wpt_y],'k*');
plot(wpt_x(1,end),wpt_y(1,end),'k');
axis([-150 200 -150 200])
axis square