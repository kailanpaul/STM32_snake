clc
clear
close all

% Define constants
A = pi/6;
omega = pi;
phi = -2/5 * pi * 0.5;
joints = 0:12;
frame_number = 1;
h = figure;
time_step = 0.1;
fps = 1/time_step;
max_val = 0;
max_curr = 0;

v = VideoWriter("do_the_worm.avi");
v.FrameRate = fps;
open(v)
h.Visible = 'off';
for t=0:time_step:10
    points = [];
    m = [];
    alpha_desired = [];
    for i=1:13 
        points_i = sin(t*omega + phi*(i-1));
        points(i) = points_i;
    end
    plot(joints, points, 'o-')
    frame = getframe(gcf);
    M(frame_number) = frame;
    frame_number = frame_number + 1;
    writeVideo(v,frame)
end
h.Visible = 'on';
movie(M, 2, fps);
close(v)


