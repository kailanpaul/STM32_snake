clc
clear
close all

% Define constants
A = pi/6;
omega = pi;
phi = -2/5 * pi;
joints = 0:11;
frame_number = 1;
h = figure;
time_step = 0.1;
fps = 1/time_step;

v = VideoWriter("do_the_worm.avi");
v.FrameRate = fps;
open(v)

h.Visible = 'off';
for t=0:time_step:10
    alpha_desired = [];
    for j=1:12 
        alpha_desired_j = sin(t*omega + phi*(j-1));
        alpha_desired(j) = alpha_desired_j;
    end
    plot(joints, alpha_desired, 'o-')
    frame = getframe(gcf);
    M(frame_number) = frame;
    frame_number = frame_number + 1;
    writeVideo(v,frame)
end

h.Visible = 'on';
movie(M, 2, fps);

close(v)