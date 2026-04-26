% path_manager_straight_line
%   - follow straight_line airplane paths between waypoint configurations
%
% Modified:  
%   - 12/6/2025 - SZM
%   - 12/6/2025 - SZM
%
% input is:
%   num_waypoints - number of waypoint configurations
%   waypoints    - an array of dimension 5 by P.size_waypoint_array.
%                - the first num_waypoints rows define waypoint
%                  configurations
%                - format for each waypoint configuration:
%                  [wn, we, wd, chi_d, Va_d]
%                  where the (wn, we, wd) is the NED position of the
%                  waypoint, chi_d is the desired course at the waypoint,
%                  and Va_d is the desired airspeed along the path. 
%
% output is:
%   flag - if flag==1, follow straight-line path
%          if flag==2, follow spiral path 
%   Va_d - desired airspeed
%   c    - either an inertial point on stright-line or center of the spiral
%   psi  - either heading angle of straight-line or start angle for spiral
%   gam  - flight path angle for straight-line and spiral
%   rho  - radius of spiral
%   lambda = direction of spiral (+1 for CW, -1 for CCW)
%
function out = path_manager_straight_line(in,P,start_of_simulation)

  NN = 0;
  num_waypoints = in(1+NN);
  waypoints = reshape(in(2+NN:5*P.size_waypoint_array+1+NN),...
                         5,P.size_waypoint_array);
  NN = NN + 1 + 5*P.size_waypoint_array;
  pn        = in(1+NN);
  pe        = in(2+NN);
  h         = in(3+NN);
  % Va      = in(4+NN);
  % alpha   = in(5+NN);
  % beta    = in(6+NN);
  % phi     = in(7+NN);
  % theta   = in(8+NN);
  chi     = in(9+NN);
  % p       = in(10+NN);
  % q       = in(11+NN);
  % r       = in(12+NN);
  % Vg      = in(13+NN);
  % wn      = in(14+NN);
  % we      = in(15+NN);
  % psi     = in(16+NN);
  state     =  in(1+NN:16+NN);
  NN = NN + 16;
  t         = in(1+NN);
 
  p = [pn; pe; -h];

  persistent waypoints_old   % stored copy of old waypoints
  persistent ptr_a           % waypoint pointer
  persistent state_transition % state of transition state machine
  persistent dubinspath
  persistent flag_need_new_waypoints % flag that request new waypoints from path planner
  persistent flag_first_time_in_state
  persistent halfplane
  persistent required_crossing
  
  if start_of_simulation,
      waypoints_old = zeros(5,P.size_waypoint_array);
      flag_need_new_waypoints = 0;
      state_transition = 0;
      flag_first_time_in_state = 1;
  end
  
  
  % if the waypoints have changed, update the waypoint pointer and plan new
  % Dubin's path
  if min(min(waypoints==waypoints_old))==0,
      waypoints_old = waypoints;
      state_transition = 1;
      ptr_a = 1;
      ptr_b = 2;
      start_node = [waypoints(1:4,ptr_a)', 0, 0];
      end_node   = [waypoints(1:4,ptr_b)', 0, 0];  
      dubinspath = dubinsParameters(start_node, end_node, P.R_min, P.gam_max);
      flag_need_new_waypoints = 0;
      flag_first_time_in_state = 1;
  end
  
  % following straight line
  switch state_transition,
      case 0, % beginning of simulation
          flag   = 1;
          Va_d   = P.Va0;
          c      = p;
          psi    = chi;
          gam    = 0;
          R      = 0;
          lambda = 0;
          if flag_first_time_in_state,
              flag_first_time_in_state =0;
          end

      case 1,
          flag   = 1;  % line
          Va_d   = waypoints(5,ptr_a); % desired airspeed along waypoint path
          c      = (dubinspath.p_s + dubinspath.p_e)/2;
          q      = dubinspath.p_e - dubinspath.p_s;
          psi    = atan2(q(2),q(1));
          gam    = -atan(q(3)/norm(q(1:2)));
          R      = dubinspath.R;
          lambda = 0;
          
          % 如果能足够接近目标点则切换到下一个目标点则切换状态
          distance = norm( p(1:2) - dubinspath.p_e(1:2) );
          % 接近50m时认为触及到目标点
          R_touch = 15;

          if distance <= R_touch
              % increase the waypoint pointer
              if ptr_a==num_waypoints-1,
                  flag_need_new_waypoints = 1;
              else
                  ptr_a = ptr_a+1;
                  ptr_b = ptr_a+1;
                  % plan new Dubin's path to next waypoint configuration
                  start_node = [waypoints(1:4,ptr_a)', 0, 0];
                  end_node   = [waypoints(1:4,ptr_b)', 0, 0];      
                  dubinspath = dubinsParameters(start_node, end_node, P.R_min, P.gam_max);    
                  state_transition = 1;
                  flag_first_time_in_state = 1;
              end
          end
   end

   out = [flag; Va_d; c; psi; gam; R; lambda; state; flag_need_new_waypoints];
  
   return;
  
end

