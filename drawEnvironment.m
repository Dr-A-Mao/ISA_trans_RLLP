function drawEnvironment(uu,V,F,colors,map,R_min)

    % process inputs to function
    NN = 0;
    pn       = uu(1+NN);       % inertial North position     
    pe       = uu(2+NN);       % inertial East position
    pd       = uu(3+NN);       % inertial Down position
    u        = uu(4+NN);       % body frame velocities
    v        = uu(5+NN);       
    w        = uu(6+NN);       
    phi      = uu(7+NN);       % roll angle         
    theta    = uu(8+NN);       % pitch angle     
    psi      = uu(9+NN);       % yaw angle     
    p        = uu(10+NN);      % roll rate
    q        = uu(11+NN);      % pitch rate     
    r        = uu(12+NN);      % yaw rate    
    t        = uu(13+NN);      % time
    
    NN = NN + 13;
    path     = uu(1+NN:9+NN); 
    NN = NN + 9;
    num_waypoints = uu(1+NN);
    waypoints     = reshape(uu(2+NN:5*num_waypoints+1+NN),5,num_waypoints)'; 


    % define persistent variables 
    persistent aircraft_handle;  % figure handle for MAV
    persistent path_handle;      % handle for straight-line or orbit path
    persistent waypoint_handle;  % handle for waypoints

    S = 500; % plot size
    
    % first time function is called, initialize plot and persistent vars
    if t==0,
        figure;clf;
        aircraft_handle = drawBody(V,F,colors,...
                                   pn,pe,pd,phi,theta,psi,...
                                   [], 'normal');
        hold on
        % 画实际的dubins航迹点
        path_handle = drawPath(path, S/2, [], 'normal');
        drawMap(map);
        % 画参考航迹点
        plot3(waypoints(:,2),waypoints(:,1),-waypoints(:,3),...
              'LineWidth',1.5,'Color','b',...
              'LineStyle','--','Marker','x',...
              'MarkerEdgeColor','r');
        title('UAV');
        xlabel('East');ylabel('North'); zlabel('Height');

        x_min = min(waypoints(:,2)); x_max = max(waypoints(:,2));
        y_min = min(waypoints(:,1)); y_max = max(waypoints(:,1));
        z_min = min(waypoints(:,3)); z_max = max(waypoints(:,3));
%         axis([x_min,x_max,y_min,y_max,z_min,z_max]);
        axis([-map.width/5,map.width * 1.28,...
              -map.width/5,map.width * 1.28,...
              0,150]);
        view(-40,70)  % set the view angle for figure
        grid on
    % at every other time step, redraw MAV
    else 
        drawBody(V,F,colors,...
                     pn,pe,pd,phi,theta,psi,...
                     aircraft_handle);
        drawPath(path, S, path_handle);
    end
end  