close all; 
clc

%goal
g = [0;30];

%arms
l1 = 10;
l2 = 10;
l3 = 10;

%starting angles

t1 = pi/4;
t2 = 3*pi/2;
t3 = 2*pi/3;

t1_limits = [-360,360];
t2_limits = [-360,360];
t3_limits = [-360,360];

t1_angle = ((t1_limits(2) - t1_limits(1))/2) * pi/180;
t2_angle = ((t2_limits(2) - t2_limits(1))/2) * pi/180;
t3_angle = ((t3_limits(2) - t3_limits(1))/2) * pi/180;

desired_angles = [t1_angle; t2_angle; t3_angle];

gain = [0,0,0;0,0,0;0,0,0];
        
dist = 3;

speed = 0.5;

while 1   
    fprintf('beginning \n');

    while dist >= 1

        p1 = [l1*cos(t1);l1*sin(t1)];
        p2 = [l1*cos(t1)+l2*cos(t1+t2);l1*sin(t1)+l2*sin(t1+t2)];
        ee = [l1*cos(t1)+l2*cos(t1+t2)+l2*cos(t1+t2+t3);l1*sin(t1)+l2*sin(t1+t2)+l3*sin(t1+t2+t3)];

        V = [g(1)-ee(1); g(2)-ee(2)];

        line1 = line([0 p1(1)],[0 p1(2)] ,'Color','blue') ;
        line2 = line([p1(1) p2(1)],[p1(2) p2(2)] ,'Color','red') ;
        line3 = line([p2(1) ee(1)],[p2(2) ee(2)] ,'Color','green') ;
        axis([-50 50 -50 50])

        axis square
        drawnow;


        if exist('line1', 'var')
         delete(line1);
         delete(line2);
         delete(line3);
        end

        J = [-l1*sin(t1)-l2*sin(t1+t2)-l3*sin(t1+t2+t3),...
        -l2*sin(t1+t2)-l3*sin(t1+t2+t3),...
        -l3*sin(t1+t2+t3);...
        l1*cos(t1)+l2*cos(t1+t2)+l3*cos(t1+t2+t3),...
        l2*cos(t1+t2)+l3*cos(t1+t2+t3),...
        l3*cos(t1+t2+t3)];  

        current_angles = [t1;t2;t3];
        z = gain*((current_angles-desired_angles).^2);
        b = inv(J*transpose(J))*(V + J*z);
        theta = transpose(J)*b - z;


        dt1 = theta(1);
        dt2 = theta(2);
        dt3 = theta(3);

        t1 = t1 + dt1*speed;
        t2 = t2 + dt2*speed;
        t3 = t3 + dt3*speed;
        
        degrees1 = int16(t1*(180/pi));
        degrees2 = int16(t2*(180/pi));
        degrees3 = int16(t3*(180/pi));
        fprintf('angles %d %d %d \n', degrees1, degrees2, degrees3);

        vx2 = double(V(1).^2);
        vy2 = double(V(2).^2);

        dist_prev = dist;
        dist = uint8(sqrt(vx2 + vy2));
        fprintf('current dist %d \n', dist);
        
        if (abs(dist - dist_prev) < 0.0001 * dist_prev)
            fprintf('break \n');
            break;
        end      
    end
    
    fprintf('end \n');
    line1 = line([0 p1(1)],[0 p1(2)] ,'Color','blue') ;
    line2 = line([p1(1) p2(1)],[p1(2) p2(2)] ,'Color','red') ;
    line3 = line([p2(1) ee(1)],[p2(2) ee(2)] ,'Color','green') ;
    axis([-50 50 -50 50])
    axis square

    target = inputdlg({'x','y', 'velocity'},'Target', [1 10; 1 10;1 20]);
 
    g = [str2double(target(1));str2double(target(2))];
    speed = str2double(target(3));
    V = [g(1)-ee(1); g(2)-ee(2)];
    vx2 = double(V(1).^2);
    vy2 = double(V(2).^2); 
    dist = uint8(sqrt(vx2 + vy2));
    fprintf('new dist %d \n', dist);
    
    
    if exist('line1', 'var')
     delete(line1);
     delete(line2);
     delete(line3);
    end
    
    t1_limits = [-360,360];
    t2_limits = [-360,360];
    t3_limits = [-10,10];

    t1_angle = ((t1_limits(2) - t1_limits(1))/2) * pi/180;
    t2_angle = ((t2_limits(2) - t2_limits(1))/2) * pi/180;
    t3_angle = ((t3_limits(2) - t3_limits(1))/2) * pi/180;

    desired_angles = [t1_angle; t2_angle; t3_angle];
    
    t1_gain = 50/(t1_limits(2) - t1_limits(1));
    t2_gain = 50/(t2_limits(2) - t2_limits(1));
    t3_gain = 50/(t3_limits(2) - t3_limits(1));
    
    if (t1_limits(2) - t1_limits(1)) > 360
        t1_gain = 0;
    end
    if (t2_limits(2) - t2_limits(1)) > 360
        t2_gain = 0;
    end
    if (t2_limits(2) - t2_limits(1)) > 360
        t2_gain = 0;
    end

    gain = [t1_gain,0,0;0,t2_gain,0;0,0,t3_gain];
end    

