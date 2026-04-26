%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function handle = drawWaypoints(waypoints, R_min, handle, mode)

    if waypoints(1,4)==-9999, % check to see if Dubins paths
        XX = [waypoints(:,1)];
        YY = [waypoints(:,2)];
        ZZ = [waypoints(:,3)];
    else
        XX = [];
        YY = [];
        for i=2:size(waypoints,1),
            dubinspath = dubinsParameters(waypoints(i-1,:),waypoints(i,:),R_min);
            [tmpX,tmpY] = pointsAlongDubinsPath(dubinspath,0.1);
            XX = [XX; tmpX];
            YY = [YY; tmpY];     
        end
        ZZ = waypoints(i,3)*ones(size(XX));
    end
    
    if isempty(handle),
        handle = plot3(YY,XX,-ZZ,'b-', 'EraseMode', mode);
    else
        set(handle,'XData', YY, 'YData', XX, 'ZData', -ZZ);
        drawnow
    end
end 