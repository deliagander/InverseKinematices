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
t2 = pi/2;
t3 = 2*pi/3;
        
dist = 1;

speed = 0.1;

while 1   
    fprintf('beginning \n');

    while dist >= 1
        % calculate joint locations
        p1 = [l1*cos(t1);l1*sin(t1)];
        p2 = [l1*cos(t1)+l2*cos(t1+t2);l1*sin(t1)+l2*sin(t1+t2)];
        ee = [l1*cos(t1)+l2*cos(t1+t2)+l2*cos(t1+t2+t3);l1*sin(t1)+l2*sin(t1+t2)+l3*sin(t1+t2+t3)];

        
        V = [g(1)-ee(1); g(2)-ee(2)];
        
        % calculate lines
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

        % calculate Jacobian
        J = [-l1*sin(t1)-l2*sin(t1+t2)-l3*sin(t1+t2+t3),...
        -l2*sin(t1+t2)-l3*sin(t1+t2+t3),...
        -l3*sin(t1+t2+t3);...
        l1*cos(t1)+l2*cos(t1+t2)+l3*cos(t1+t2+t3),...
        l2*cos(t1+t2)+l3*cos(t1+t2+t3),...
        l3*cos(t1+t2+t3)];  

        % damped least sqaures calculation
        lamda = speed*40;
        theta = (transpose(J)*J+lamda.^2*eye(3))\(transpose(J)*V);


        dt1 = theta(1);
        dt2 = theta(2);
        dt3 = theta(3);
        
        % update joint angles
        t1 = t1 + dt1*speed;
        t2 = t2 + dt2*speed;
        t3 = t3 + dt3*speed;

        vx2 = double(V(1).^2);
        vy2 = double(V(2).^2);

        dist_prev = dist;
        dist = sqrt(vx2 + vy2);
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
    dist = sqrt(vx2 + vy2);
    fprintf('new dist %d \n', dist);
    
    
    if exist('line1', 'var')
     delete(line1);
     delete(line2);
     delete(line3);
    end
end    

