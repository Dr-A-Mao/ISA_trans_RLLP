%% 定义UCAV类
classdef UCAV < handle
    properties
        %% 重要变量
        lon_scale
        lat_scale
        alive % 当前存活状态
        vel % 速度
        MaxAngle % 最大转弯角
        traj_length % 轨迹长度
        refpath_length % 参考路径长度
        max_distance % 路径点到终点最大直线距离
        current_task %当前任务索引
        current_lng
        current_lat
        current_hei
        init_lng % 当前经度
        init_lat % 当前纬度
        eta_lon
        eta_lat
        % 任务属性
        dist
        time
        cost
        % 任务属性序列
        Tasks % 任务索引序列,维度与端点相同
        Paths % 路径序列，比端点-1，维度与边相同
        path_ref
        ref_x_c
        ref_y_c
        ref_z_c
        neighbor_indexs % 相邻结点的索引
        neighbor_betas % 相邻结点的连接强度
        neighbor_ks % 成型的强度
        % 用于拍卖的任务属性
        task_star
        path_star
        dist_star
        time_star
        cost_star  
        % 当前的运动学状态量
        xr
        yr
        zr
        chi
        gamma
        Vg
        Va
        % 重要的参数
        Vg_range   % Vg范围
        a_bz_range % a_bz范围
        n_lf_range
        phi_range  % phi范围
        a_yc_range % a_yc范围
        a_zc_range % a_zc范围
        gamma_range % gamma范围
        chi_range  % chi范围
        P % 结构化参数P
        out % 输出结构
        g
        delta_t 
        t 
    end
    methods
        %% 初始化函数
        function obj = UCAV(current_lng,current_lat,delta_t,vel,MaxAngle)
            if nargin == 2
                vel = 50;
                MaxAngle = 15;
                delta_t = 1;
            end
            obj.lon_scale = 110e3;
            obj.lat_scale = 113e3;
            obj.vel = vel;
            obj.alive = 1;
            obj.MaxAngle = MaxAngle;
            obj.init_lng = current_lng;
            obj.init_lat = current_lat;
            obj.current_lng = current_lng;
            obj.current_lat = current_lat;
            obj.current_hei = 1.201e+03;
            obj.current_task = 0; % 当前任务为0
            % 关键变量
            obj.eta_lon = 0;
            obj.eta_lat = 0;
            obj.dist = 0;
            obj.time = 0;
            obj.cost = 0;
            % 任务序列
            obj.Tasks = Datastr();
            obj.Paths = Datastr();
            obj.path_ref = [];
            obj.ref_x_c = [];
            obj.ref_y_c = [];
            obj.ref_z_c = [];
            obj.neighbor_indexs = [];
            obj.neighbor_betas = [];
            obj.neighbor_ks = [];
            % 拍卖任务
            obj.task_star = Datastr();
            obj.path_star = Datastr();
            obj.dist_star = 0;
            obj.time_star = 0;
            obj.cost_star = 0;
            obj.traj_length = 0;
            obj.refpath_length = 0;
            obj.max_distance = 0;
            % 运动学状态量
            obj.xr = 0;
            obj.yr = 0;
            obj.zr = 1.201e+03;
            obj.chi = 0;
            obj.gamma = 0;
            % 重要的超参数
            obj.Vg = vel; % 默认设置为小型固定翼无人机的速度为30m/s
            obj.Va = vel; % 空速，默认和地速一致
            obj.g = 9.8; % 重力加速度为9.8m^2/s
            obj.t = 0; % 当前的时刻从0开始
            obj.delta_t = delta_t; % 默认的采样时间
            obj.Vg_range = [9,16];
            obj.gamma_range = [-pi/2,pi/2];
            obj.chi_range = [-pi/2,pi/2];
            obj.a_bz_range = [6,25];
            obj.phi_range = [-0.6,0.6];
            obj.n_lf_range = [0,2.1];
            obj.a_yc_range = [-obj.a_bz_range(2) * cos(obj.phi_range(1)),...
                              obj.a_bz_range(2) * cos(0)];
            obj.a_zc_range = [obj.a_bz_range(1) * sin(obj.phi_range(1)),...
                              obj.a_bz_range(2) * sin(obj.phi_range(2))];
            % 初始化参数P
            % param;
            % evalin('base','P');
            obj.P = [];
            obj.out = struct();
        end
        %% 获取路径函数
        % 用以确定UAV任务在第k任务和第k+1个任务之间
        function k = decide_task_index(obj,task_lat,task_lng)
            % k必须小于N，用于计算当前要插入第几个路径
            num_path = obj.Paths.length;
            if num_path == 0
                k = 0;
                return;
            end
            % 遍历查找最近的路径点
            distance_array = zeros(1,num_path);
            for i = 1:num_path
                path_i = obj.Paths.data{i};
                left_path_lng = path_i(1,1);
                left_path_lat = path_i(1,2);
                %left_path_hei = path_i(1,3);

                right_path_lng = path_i(end,1);
                right_path_lat = path_i(end,2);
                %right_path_hei = path_i(end,3); 

                path_distance = obj.cal_path_length(path_i);
                % 声明比例系数
                epsilon1 = 0.2; epsilon2 = 0.2;

                left_distance =  (1 + epsilon1) * norm([( left_path_lng - task_lng) * obj.lon_scale,...
                                      ( left_path_lat - task_lat) * obj.lat_scale,...
                                      ]);

                right_distance = (1 + epsilon2) * norm([-( right_path_lng - task_lng) * obj.lon_scale,...
                                       -( right_path_lat - task_lat) * obj.lat_scale,...
                                       ]);

                distance_array(i) = (left_distance + right_distance) - path_distance;
                % 获取终点坐标
                if i == num_path
                    end_distance = right_distance;
                end
            end
            % 计算最短的点
            [min_distance,k] = min(distance_array);
            % 再对比最短距离，若距离更短则插入到最后一个
            epsilon1 = 0.2;
            if (1 + epsilon1) * end_distance <= min_distance 
                k = -1; % k=-1代表从当前路径最后一点插入
            end
        end
        %% 用于执行当k=-1时的插入操作
        function inset_end_node(obj,task_lng,task_lat,task_index)
            % k=-1意味着结点插入是在最后状态的插入
            % 先克隆2个Tasks和Paths
            Tasks_clone = obj.Tasks.clone();
            Paths_clone = obj.Paths.clone();
            % 当前点经纬度
            path_end = Paths_clone.data{end};
            start_lng = path_end(end,1);
            start_lat = path_end(end,2);
            % 计算得到从最后点到任务点的路径
            [path_new,dist_new,time_new,cost_new] = obj.get_path_info(start_lng,start_lat,...
                                                                      task_lng,task_lat);
            % 结点替补信息的修改
            obj.dist_star = obj.dist + dist_new;
            obj.time_star = obj.time + time_new;
            obj.cost_star = obj.cost + cost_new;
            % 在最后一个结点处进行插入
            Tasks_clone.append_elements(task_index);
            obj.task_star = Tasks_clone.clone();
            % 对克隆体Paths_clone进行操作
            Paths_clone.append_elements(path_new);
            obj.path_star = Paths_clone.clone();
        end

        %% 用以获取第k个path的路径参数,k足够小,这个函数很重要，用于决定任务序列
        function [path_k,dist_k,time_k,cost_k] = get_param_path(obj,k) 
            % 计算第k条路径
            path_k = obj.Paths.data{k};
            % 根据path_k计算相应参数
            path_diff_lon_k = diff(path_k(:,1)) * obj.lon_scale;
            path_diff_lat_k = diff(path_k(:,2)) * obj.lat_scale;
            path_diff_alt_k = diff(path_k(:,3)) * 1;
            % 计算中间结点
            dist_k = norm([path_diff_lon_k,path_diff_lat_k,path_diff_alt_k]);
            time_k = dist_k/obj.vel;
            cost_k = dist_k; % 默认cost_star为dist_star这里可能要改!!!!!!!!!!!
        end
        %% 下面函数用于计算拍卖量Star
        function update_auction(obj,task_lng,task_lat,task_index)
             % task_lat,task_lng: 新任务的位置, task_index: 新任务索引
             % 获取插入序列索引
             k = obj.decide_task_index(task_lat,task_lng); % 获得当前任务T的插入索引
             % 若当前并没有其他路径时
             if k == 0
                 [path_new,dist_new,time_new,cost_new] = obj.get_path_info(obj.init_lng,obj.init_lat,...
                                                                           task_lng,task_lat);
                 obj.dist_star = dist_new;
                 obj.time_star = time_new;
                 obj.cost_star = cost_new;
                 % 对克隆体Tasks_clone进行操作
                 Tasks_clone = obj.Tasks.clone();
                 Paths_clone = obj.Paths.clone();
                 % 对克隆体插入0元素
                 Tasks_clone.insert_elements(k,task_index);
                 obj.task_star = Tasks_clone.clone();
                 Paths_clone.insert_elements(k,path_new);
                 obj.path_star = Paths_clone.clone();
                 return;
             end
             % 若当前需要将路径插入到当前路径最后
             if k == -1
                obj.inset_end_node(task_lng,task_lat,task_index); % 嵌入到路径最末端
                return;
             end
             [path_k,dist_k,time_k,cost_k] = obj.get_param_path(k);
             % 确定path_k的起点和终点
             path_k_beginlon = path_k(1,1); path_k_beginlat = path_k(1,2);  % k个点
             path_k_endlon = path_k(end,1); path_k_endlat = path_k(end,2);  % k+1 个点
             % 根据以上信息获取重要属性
             [path_new_task1,dist_new_task1,time_new_task1,cost_new_task1] = obj.get_path_info(path_k_beginlon,path_k_beginlat, ...
                                                                                               task_lng, task_lat);
             [path_new_task2,dist_new_task2,time_new_task2,cost_new_task2] = obj.get_path_info(task_lng,task_lat, ...
                                                                                               path_k_endlon, path_k_endlat);
             % 中继标量插入
             obj.dist_star = obj.dist - dist_k + dist_new_task1 + dist_new_task2;
             obj.time_star = obj.time - time_k + time_new_task1 + time_new_task2;
             obj.cost_star = obj.cost - cost_k + cost_new_task1 + cost_new_task2;
             % 先克隆2个Tasks和Paths
             Tasks_clone = obj.Tasks.clone();
             Paths_clone = obj.Paths.clone();
             % 对克隆体Tasks_clone进行操作
             Tasks_clone.insert_elements(k,task_index);
             obj.task_star = Tasks_clone.clone();
             % 对克隆体Paths_clone进行操作
             Paths_clone.modify_elements(k,path_new_task1);
             Paths_clone.insert_elements(k,path_new_task2); % 这里不太确定后面可能得改
             obj.path_star = Paths_clone.clone();
        end
        %% 获取中间路径属性
        function [path_temp,dist_temp,time_temp,cost_temp] = get_path_info(obj,task_beginlng,task_beginlat,task_endlng,task_endlat)
            [path_temp,dist_temp,time_temp,cost_temp] = get_path_temp(task_beginlat,...
                                                        task_beginlng,task_endlat,...
                                                        task_endlng,obj.MaxAngle,obj.vel);
        end
        %% 若被接受则改变当前的
        function accept(obj)
            obj.dist = obj.dist_star;
            obj.time = obj.time_star;
            obj.cost = obj.cost_star;
            obj.Paths = obj.path_star.clone();
            obj.Tasks = obj.task_star.clone();
            obj.joint_paths(); % 获得path_ref
        end
        %% 用于计算拍卖代价
        % key points to be considered is :
        function cost = get_cost(obj)
            % 用以计算拍卖收益
           cost = obj.cost_star; 
        end
        %% 计算当前点和目标点的theta
        function theta = cal_theta(obj,flag)
            % 总的参考飞行路径
            path = obj.path_ref;
            if strcmp(flag,'distance')    
                % 当前点和目标点的直线距离
                theta = norm([(obj.current_lng - path(end,1))*obj.lon_scale, ...
                              (obj.current_lat - path(end,2))*obj.lat_scale, ...
                               obj.current_hei - path(end,3)]);
            elseif strcmp(flag,'integrated')
                alpha_1 = 0.7;
                alpha_2 = 0.3;
                % 指标的综合
                theta_1 = 1 - obj.traj_length / (obj.refpath_length + 1e-5);
                theta_2 = norm([(obj.current_lng - path(end,1))*obj.lon_scale, ...
                              (obj.current_lat - path(end,2))*obj.lat_scale, ...
                               obj.current_hei - path(end,3)])/obj.max_distance;
                theta = alpha_1 * theta_1 + alpha_2 * theta_2;
            elseif strcmp(flag,'time')
                waypoints = obj.P.waypoints;

                pn_c = obj.P.pn_c;
                pe_c = obj.P.pe_c;
                pz_c = obj.P.pz_c;

                path_ref_x = waypoints(:,1); 
                path_ref_y = waypoints(:,2); 
                path_ref_z = waypoints(:,3);
                % 计算当前点到目标点的长度
                d_c = norm([pn_c - obj.xr, ...
                            pe_c - obj.yr, ...
                            pz_c - obj.zr]);

                [~,n] = min((path_ref_x - pn_c).^2 + ...
                            (path_ref_y - pe_c).^2 + ...
                            (path_ref_z - pz_c).^2);
                % 计算航路端长度
                sum_l_P = 0;
                for k = n+1:length(path_ref_x)
                    sum_l_P = sum_l_P + sqrt((path_ref_x(k) - path_ref_x(k-1))^2 + ...
                                             (path_ref_y(k) - path_ref_y(k-1))^2 + ...
                                             (path_ref_z(k) - path_ref_z(k-1))^2);
                end
                theta = (d_c + sum_l_P)/obj.Vg;
            else
                % 当前点和目标点路程的比例
                theta = 1 - obj.traj_length / (obj.refpath_length + 1e-5);
            end
        end
        %% 聚合相邻结点的theta计算自己的理想的theta_c
        function theta_c = aggr_neighbor_thetas(obj,UCAV_array,flag)
            % 函数function 
            f = @(x)tanh(x);
            % 得到beta_array
            beta_array = obj.neighbor_betas;
            % 得到neighbor_index_array
            neighbor_index_array = obj.neighbor_indexs;
            % 得到k_theta
            theta_max = 4;
            k_theta = 4 / theta_max;
            % 计算自己的theta
            theta_i = obj.cal_theta(flag);
            % 聚合相邻结点的thetas
            theta_j_array = zeros(1,length(neighbor_index_array));
            for k = 1 : length(neighbor_index_array)
                theta_j_array(k) = UCAV_array(neighbor_index_array(k)).cal_theta(flag);
            end
            theta_dot = - sum(beta_array .* ...
                              f(k_theta * (theta_i - theta_j_array)));
            % 计算参考的theta_c
            theta_c = theta_i + theta_dot * obj.delta_t;
        end
        %% 计算出当前的速度制导率
        function Vg_c = guidance_Vg(obj,theta_c,theta,k_Vg)
            Vg_dot_c = -k_Vg * (theta_c - theta);
            % 状态更新
            Vg_c = obj.Vg + obj.delta_t * Vg_dot_c;
            % 进行限幅
            Vg_c = obj.clip(Vg_c,obj.Vg_range(1),obj.Vg_range(2));
            %disp(['Vg_dot_c:',num2str(Vg_dot_c),',Vg_c:',num2str(Vg_c)]);
        end
        %% 用于计算具体路径的长度
        function path_length = cal_path_length(obj)
            path = obj.path_ref;
            path_x_diff = diff(path(:,1) * obj.lon_scale);
            path_y_diff = diff(path(:,2) * obj.lat_scale);
            path_z_diff = diff(path(:,3));
            path_length = sum(sqrt(path_x_diff.^2 + path_y_diff.^2 + path_z_diff.^2));
            obj.refpath_length = path_length;
            %计算当前点到目标点的距离最大值
            path_dis = path - path(end,:);
            distances = sqrt((path_dis(:,1) * obj.lon_scale).^2 + ...
                             (path_dis(:,2) * obj.lat_scale).^2 + ...
                             (path_dis(:,3)).^2);
            obj.max_distance = max(distances);
        end
        %% 合并UAV的所有paths
        function path = joint_paths(obj)
            num_paths = length(obj.Paths.data);
            path = [];
            for k = 1:num_paths
                path = [path;...
                        obj.Paths.data{k}];
            end
            obj.path_ref = path;
        end
        %% 下面根据参考的path_ref和当前的(xr,yr,zr)得到待跟踪的目标点
        function [x,y,z] = lng_lat_hei_to_xyz(obj,lng,lat,hei)
            x = (lng - obj.init_lng) * obj.lon_scale;
            y = (lat - obj.init_lat) * obj.lat_scale;
            z = hei;
        end
        function [lng,lat,hei] = xyz_to_lng_lat_hei(obj,x,y,z)
            lng = x/obj.lon_scale + obj.init_lng;
            lat = y/obj.lat_scale + obj.init_lat;
            hei = z;
        end
        function [phi_c,n_lf_c] = a_yc_zc_to_phi_n_lf(obj,a_yc,a_zc)
            phi_c = obj.clip(asin(a_yc/obj.g),obj.phi_range(1),obj.phi_range(2));
            n_lf_c = obj.clip(a_zc/(obj.g * cos(phi_c)),obj.n_lf_range(1),obj.n_lf_range(2));
        end
        %% 下面确定被跟踪的点
        function [x_p,y_p,z_p] = get_track_point2(obj)
            % 得到关键参量
            x_r = obj.xr; y_r = obj.yr; z_r = obj.zr;
            Vg_r = obj.Vg; gamma_r = obj.gamma; chi_r = obj.chi;
            [x_p,y_p,z_p] = get_track_point(obj.P,x_r,y_r,z_r,Vg_r,gamma_r,chi_r);
            % 赋值
            obj.P.pe_c = x_p;
            obj.P.pn_c = y_p;
            obj.P.pz_c = z_p;

            obj.ref_x_c = x_p;
            obj.ref_y_c = y_p;
            obj.ref_z_c = z_p;
        end

        function [x_p,y_p,z_p] = get_track_point(obj,path_x,path_y,path_z)
            % 得到关键参量
            x_r = obj.xr; y_r = obj.yr; z_r = obj.zr;
            Vg_r = obj.Vg; gamma_r = obj.gamma; chi_r = obj.chi;
            
            if isempty(obj.Va)
                Vg_r = obj.Vg_range(1);
            else
                Vg_r = obj.Va;
            end

            % 路径类型
            path_type = 1;
            % 找到其中满足约束的路径点
                if path_type == 1
                    % 路径参量
                    path_eta_lat = atan2(path_y - y_r,path_x - x_r) - chi_r;
                    path_eta_lon = atan2(path_z - z_r, sqrt((path_x - x_r).^2 + (path_y - y_r).^2)) - gamma_r;
                    path_index =  find((abs(path_eta_lat) <= pi/2) & (abs(path_eta_lon) <= pi/2));
                    if ~isempty(path_index)
                        path_x = path_x(path_index);
                        path_y = path_y(path_index);
                        path_z = path_z(path_index);
                    end
                    % 搜索在路径path中最近投影点
                    distance_r_path = sqrt((path_x-x_r).^2 + ...
                                           (path_y-y_r).^2 + ...
                                           (path_z-z_r).^2);
                    [~,i] = min(distance_r_path); % 投影点索引为i
                    next_distance_array = distance_r_path(i:end);
                    % 下面计算重要的制导率
                    q_L = obj.delta_t;
                    L = q_L * Vg_r;
                    %% 需要修改为待跟踪的轨迹
                    % 计算下一点的索引
                    [~,k] = min(abs(next_distance_array - L));
                    %待跟踪结点索引 
                    p_index = i + k - 1;
                    x_p = path_x(p_index);
                    y_p = path_y(p_index);
                    z_p = path_z(p_index);
                elseif path_type == 2
                    path_eta_lat = atan2(path_y - y_r,path_x - x_r) - chi_r;
                    [~,p_index] = min(abs(path_eta_lat));
                    x_p = path_x(p_index);
                    y_p = path_y(p_index);
                    z_p = path_z(p_index); 
                elseif path_type == 3
                    path_eta_lon = atan2(path_z - z_r, sqrt((path_x - x_r).^2 + (path_y - y_r).^2)) - gamma_r;
                    [~,p_index] = min(abs(path_eta_lon));
                    x_p = path_x(p_index);
                    y_p = path_y(p_index);
                    z_p = path_z(p_index); 
                end
            % 赋值
            obj.ref_x_c = x_p;
            obj.ref_y_c = y_p;
            obj.ref_z_c = z_p;
        end
        %% UCAV的动力学模型更新
        function out = dynamic_simulation(obj,Va_c)
            %% 更新P值
            obj.P.Va_c = Va_c;
            % 更新P值
            obj.P.pn0 = obj.out.pn.Data(end);
            obj.P.pe0 = obj.out.pe.Data(end);
            obj.P.pd0 = -obj.out.pd.Data(end);
            obj.P.u0 = obj.out.u.Data(end);
            obj.P.v0 = obj.out.v.Data(end);
            obj.P.w0 = obj.out.w.Data(end);
            obj.P.phi0 = obj.out.phi.Data(end);
            obj.P.theta0 = obj.out.theta.Data(end);
            obj.P.psi0 = obj.out.psi.Data(end);
            obj.P.p0 = obj.out.p.Data(end);
            obj.P.q0 = obj.out.q.Data(end);
            obj.P.r0 = obj.out.r.Data(end);

            %% 仿真simulink,在[0,stopTime]时间段内仿真
            % 将数据P塞到workspace中
            assignin('base','P',obj.P);
            % 开始仿真得到out
            out = sim('model2.slx', 'StopTime', num2str(obj.delta_t));
            %% 重新赋值P
            obj.out = out;
            %% 更新状态
            % 状态更新
            obj.xr = out.pe.Data(end);
            obj.yr = out.pn.Data(end);
            obj.zr = out.pd.Data(end); %% !!!注意out.pd.Data永远是正的
            obj.Vg = out.Vg.Data(end);
            obj.Va = out.Va.Data(end);
            obj.chi = out.chi.Data(end);
            obj.gamma = out.gamma.Data(end);
            % 更新经纬度
            obj.current_lng = obj.init_lng + obj.xr / obj.lon_scale;
            obj.current_lat = obj.init_lat + obj.yr / obj.lat_scale;
            obj.current_hei = obj.zr;
            % 更新轨迹长度
            obj.traj_length = obj.traj_length + obj.delta_t * ( mean(obj.out.Vg.Data) ); 
        end
        %% 下面删除pathref中不合理的航路点
        function delete_infeasible_ref_points(obj,polygon_3D)
            ref_lon = obj.path_ref(:,1);
            ref_lat = obj.path_ref(:,2);
            ref_hei = obj.path_ref(:,3);
            
            % 将待跟踪的obj.ref_x_c,obj.ref_y_c,obj.ref_z_c换成
            [ref_x,ref_y,ref_z] = obj.lng_lat_hei_to_xyz(ref_lon,...
                                                         ref_lat,...
                                                         ref_hei);

            % 获得obstacle
            [obs_x,obs_y,obs_z] = obj.lng_lat_hei_to_xyz(polygon_3D(:,1),...
                                                         polygon_3D(:,2),...
                                                         polygon_3D(:,3));

            obstacle.polygon = [obs_x,obs_y,obs_z];
            obstacle.height = max(obs_z);
            % 更新obj.path_ref
            num_path_ref = length(ref_lon);
            ref_lon_new = -1 * ones(num_path_ref,1);
            ref_lat_new = -1 * ones(num_path_ref,1);
            ref_hei_new = -1 * ones(num_path_ref,1);
            for i = 1:num_path_ref
                if ~InConvexPolygon([ref_x(i),ref_y(i),ref_z(i)], obstacle)
                    ref_lon_new(i) = ref_lon(i);
                    ref_lat_new(i) = ref_lat(i);
                    ref_hei_new(i) = ref_hei(i);
                end
            end

            ref_lon_new = ref_lon_new(find(ref_lon_new~=-1));
            ref_lat_new = ref_lat_new(find(ref_lat_new~=-1));
            ref_hei_new = ref_hei_new(find(ref_hei_new~=-1));

            obj.path_ref = [ref_lon_new,ref_lat_new,ref_hei_new];
        end
        %% 下面将新的航路点坐标插入到path_ref中
        function insert_point(obj,lng_p,lat_p,hei_p)
            ref_lon = obj.path_ref(:,1);
            ref_lat = obj.path_ref(:,2);
            ref_hei = obj.path_ref(:,3);
            
            % 将待跟踪的obj.ref_x_c,obj.ref_y_c,obj.ref_z_c换成
            [ref_y,ref_x,ref_z] = obj.lng_lat_hei_to_xyz(ref_lon,ref_lat,ref_hei);

            [~,k] = min((ref_x - obj.ref_x_c).^2 +...
                            (ref_y - obj.ref_y_c).^2 +...
                            (ref_z - obj.ref_z_c).^2);
            
            length_ref_path = length(ref_lon);
            if k == length_ref_path
                obj.path_ref = [obj.path_ref(1:k,:);...
                                [lng_p,lat_p,hei_p]];
            else
                obj.path_ref = [obj.path_ref(1:k,:);...
                                [lng_p,lat_p,hei_p];...
                                obj.path_ref((k+1):end,:)];
            end
        end
        %% 下面是飞行器L1制导率设计
        function [a_yc,a_zc] = guidance_L1(obj,x_p,y_p,z_p)
            % 需要跟踪的path_x,path_y,path_z
            x_r = obj.xr; y_r = obj.yr; z_r = obj.zr;
            gamma_r = obj.gamma; chi_r = obj.chi; 
            Vg_r = obj.Vg;
            % 重要超参数
            xi_L = 0.707; P_L = 1; 
            % 下面计算重要的制导率
            q_L = P_L * xi_L /pi;
            k_L = 4 * xi_L^2;  
            kq = k_L/q_L;
            % 计算eta_lat和eta_lon
            eta_lat = atan2(y_p - y_r,x_p - x_r) - chi_r;
            eta_lon = atan2(z_p - z_r,sqrt((x_p - x_r)^2 + (y_p - y_r)^2)) - gamma_r;
            obj.eta_lat = eta_lat;
            obj.eta_lon = eta_lon;
            % 下面需要考虑过载范围
            eta_lat_max = asin(obj.clip(obj.a_yc_range(2) * q_L/Vg_r/k_L,-1,1));
            eta_lat_min = asin(obj.clip(obj.a_yc_range(1) * q_L/Vg_r/k_L,-1,1));
            eta_lon_max = asin(obj.clip((obj.a_zc_range(2) - obj.g * cos(gamma_r))/kq/Vg_r,-1,1));
            eta_lon_min = asin(obj.clip((obj.a_zc_range(1) - obj.g * cos(gamma_r))/kq/Vg_r,-1,1));
            eta_lat = obj.clip(eta_lat,eta_lat_min,eta_lat_max);
            eta_lon = obj.clip(eta_lon,eta_lon_min,eta_lon_max);
            % 计算制导率
            a_yc =  kq * Vg_r * sin(eta_lat) * cos(gamma_r);
            a_yc = obj.clip(a_yc,-obj.g,obj.g);
            a_zc =  kq * Vg_r * sin(eta_lon) + obj.g * cos(gamma_r);
        end
        %% 采用MPC设计制导率
        function [a_yc,a_zc] = guidance_mpc(obj,path_x,path_y,path_z,N)
            % 该制导率目前还存在问题！！！后面还得修改...
            % 获得关键形参
            x_r = obj.xr; y_r = obj.yr; z_r = obj.zr;Vg_r = obj.Vg;
            gamma_r = obj.gamma; chi_r = obj.chi;
            x0 = [x_r,y_r,z_r,Vg_r,gamma_r,chi_r];
            % MPC
            [a_yc,a_zc] = NMPC_controller(x0,obj.delta_t,N,...
                                          path_x,path_y,path_z,...
                                          obj.a_yc_range, ...
                                          obj.a_zc_range);
        end
        %% 设计补偿策略
        function [f_lat,f_lon] = compensation(obj,x_p,y_p,z_p, ...
                                                       kq,k1,k2,...
                                                       C1_dot,C2_dot,Ld)
            % 需要跟踪的path_x,path_y,path_z
            x_r = obj.xr; y_r = obj.yr; z_r = obj.zr;
            gamma_r = obj.gamma; chi_r = obj.chi; 
            Vg_r = obj.Vg;
            % 计算eta_lat和eta_lon
            eta_lat = atan2(y_p - y_r,x_p - x_r) - chi_r;
            eta_lon = atan2(z_p - z_r,sqrt((x_p - x_r)^2 + (y_p - y_r)^2)) - gamma_r;
            % 计算上下界
            fenzi = sqrt((sin(eta_lon) * cos(eta_lon))^2 + (sin(eta_lat) * cos(eta_lat))^2);
            fenmu_lat = sin(eta_lat) * cos(eta_lat);
            fenmu_lon = sin(eta_lon) * cos(eta_lon);
            % 计算f_lon
            if abs(fenmu_lon) <= 1e-5
                f_lon = C2_dot;
            else
                f_lon = C2_dot + k2 * fenzi / fenmu_lon;
            end
            % 计算f_lat
            if abs(fenmu_lat) <= 1e-5
                f_lat = C1_dot;
            else
                f_lat = C1_dot + k1 * fenzi / fenmu_lat;
            end    
        end
        %% 设计补偿策略
        function [k1,k2] = select_k(obj,x_p,y_p,z_p,kq,...
                                        C1_dot,C2_dot,Ld)
            % 需要跟踪的path_x,path_y,path_z
            x_r = obj.xr; y_r = obj.yr; z_r = obj.zr;
            gamma_r = obj.gamma; chi_r = obj.chi; 
            Vg_r = obj.Vg; g_r = obj.g;
            a_zc_max = obj.a_zc_range(2);
            a_zc_min = obj.a_zc_range(1);
            a_yc_max = obj.a_yc_range(2);
            a_yc_min = obj.a_yc_range(1);
            % 计算eta_lat和eta_lon
            eta_lat = atan2(y_p - y_r,x_p - x_r) - chi_r;
            eta_lon = atan2(z_p - z_r,sqrt((x_p - x_r)^2 + (y_p - y_r)^2)) - gamma_r;
            % 计算上下界
            fenzi = sqrt((sin(eta_lon) * cos(eta_lon))^2 + (sin(eta_lat) * cos(eta_lat))^2);
            fenmu_lat = sin(eta_lat) * cos(eta_lat);
            fenmu_lon = sin(eta_lon) * cos(eta_lon);
            % 限幅
            f_gamma_min = kq * sin(eta_lon) + C2_dot - ( a_zc_max - g_r * cos(gamma_r) )/Vg_r;
            f_gamma_max = kq * sin(eta_lon) + C2_dot - ( a_zc_min - g_r * cos(gamma_r) )/Vg_r;
            f_chi_min = kq * sin(eta_lat) + C1_dot - a_yc_max/(Vg_r * cos(gamma_r));
            f_chi_max = kq * sin(eta_lat) + C1_dot - a_yc_min/(Vg_r * cos(gamma_r));
            % 系数
            s_chi = - fenmu_lat / fenzi;
            s_gamma = - fenmu_lon / fenzi;
            % k1,k2范围
            k1_min = min(s_chi * f_chi_min, s_chi * f_chi_max);
            k1_max = max(s_chi * f_chi_min, s_chi * f_chi_max);
            k2_min = min(s_gamma * f_gamma_min, s_gamma * f_gamma_max);
            k2_max = max(s_gamma * f_gamma_min, s_gamma * f_gamma_max);
            if k1_max + k2_max > Ld
                k1 = k1_max;
                k2 = k2_max;
            else
                k1 = [];
                k2 = [];
            end
        end
        %% 设计补偿策略修正版
        function k_opt = convex_optimal_sol(obj,R,A,b,Delta_A,Delta_b,...
                                           u_max,u_min,u_dot_max,u_dot_min,Ld)
            flag = 0; % flag = 0:能量最优，flag = 1:能量差量最优
            if flag
                H = A' * R * A;
                f = A' * R' * b;
            else
                H = Delta_A' * R * Delta_A;
                f = Delta_A' * R' * Delta_b;
            end
            A_leq = [A;-A;Delta_A;-Delta_A;[-1,-1]];
            b_leq = [u_max - b;b - u_min;...
                     u_dot_max * obj.delta_t - Delta_b;...
                     Delta_b - u_dot_min * obj.delta_t;...
                     -Ld];
            % convex optimzation
            k_opt = quadprog(H,f,A_leq,b_leq);
        end
        %% 修正版补偿策略
        function [f_lat,f_lon] = compensation_modify(obj,x_p,y_p,z_p, ...
                                                     kq,k1,k2,...
                                                     C_1_dot,C_2_dot,Ld)
            % threshold
            varepsilon = 0;
            R = diag([1,1]);
            % 需要跟踪的path_x,path_y,path_z
            x_r = obj.xr; y_r = obj.yr; z_r = obj.zr;
            gamma_r = obj.gamma; chi_r = obj.chi; 
            Vg_r = obj.Vg; 
            % 计算eta_lat和eta_lon
            eta_lat = atan2(y_p - y_r,x_p - x_r) - chi_r;
            eta_lon = atan2(z_p - z_r,sqrt((x_p - x_r)^2 + (y_p - y_r)^2)) - gamma_r;
            % 用于判断
            sin_theta = sin(eta_lat) * cos(eta_lat)/sqrt((sin(eta_lat) * cos(eta_lat))^2 + (sin(eta_lon) * cos(eta_lon))^2 + eps);
            cos_theta = sin(eta_lon) * cos(eta_lon)/sqrt((sin(eta_lat) * cos(eta_lat))^2 + (sin(eta_lon) * cos(eta_lon))^2 + eps);
            b = [(kq * sin(eta_lat) + C_1_dot) * Vg_r * cos(gamma_r);...
                 (kq * sin(eta_lon) + C_2_dot) * Vg_r + obj.g * cos(gamma_r)];
            a1 = Vg_r * cos(gamma_r)/sin_theta;
            a2 = Vg_r/cos_theta;
            % 得到最优解
            A = diag([a1,a2]);
            u_max = [obj.a_yc_range(2);obj.a_zc_range(2)];
            u_min = [obj.a_yc_range(1);obj.a_zc_range(1)];
            H = A' * R * A;
            f = A' * R' * b;
            A_leq = [A;-A;[-1,-1]];
            b_leq = [u_max - b;b - u_min;-(1+varepsilon)*Ld];
            % convex optimzation
            k_opt = quadprog(H,f,A_leq,b_leq);
            % 得到最合适的k1和k2
            if isempty(k_opt)
                [f_lat,f_lon] = obj.compensation(x_p,y_p,z_p, ...
                                                 kq,k1,k2,...
                                                 C_1_dot,C_2_dot,Ld);
            else
                disp('非空，已找到最优解！');
                 % 计算f_lon
                if abs(sin_theta) <= 5e-2
                    f_lat = C_1_dot;
                else
                    f_lat = C_1_dot +  k_opt(1)/sin_theta;
                end
                % 计算f_lat
                if abs(cos_theta) <= 5e-2
                    f_lon = C_2_dot;
                else
                    f_lon = C_2_dot + k_opt(2)/cos_theta;
                end
            end
        end
        %% 飞行器的运动学方程，这里认为是固定翼的无人机
        % 来自于文献 "Path Following for Unmanned Combat Aerial Vehicles Using
        % Three-Dimensional Nonlinear Guidance"
        function kinematic(obj,a_yc,a_zc)
            % ayc and azc are acceleration inputs to the kinematic motion generated by the guidance law
            % Vg is the inertial velocity vector of the vehicle,constant
            obj.t = obj.t + obj.delta_t;
            % 内环更新gamma、chi
            obj.chi = obj.chi + obj.delta_t * a_yc / (obj.Vg * cos(obj.gamma));
            obj.gamma = obj.gamma + obj.delta_t * (a_zc - obj.g * cos(obj.gamma))/obj.Vg;
            % 添加噪声项
            Ld = 0.0; % 噪声赋值
            obj.chi = obj.chi + Ld/sqrt(2) * (2 * rand() - 1);
            obj.gamma = obj.gamma + Ld/sqrt(2) * (2 * rand() - 1);
            % 对chi、gamma进行镇定
            while true
                if obj.chi >= pi
                    obj.chi = obj.chi - 2 * pi;
                elseif obj.chi <= -pi
                    obj.chi = obj.chi + 2 * pi;
                else
                    break;
                end
            end
            % chi限幅
            while true
                if obj.gamma >= pi
                    obj.gamma = obj.gamma - 2 * pi;
                elseif obj.gamma <= -pi
                    obj.gamma = obj.gamma + 2 * pi;
                else
                    break;
                end
            end
            %obj.chi = obj.clip(obj.chi,obj.chi_range(1),obj.chi_range(2));
            % gamma限幅
            %obj.gamma = obj.clip(obj.gamma,obj.gamma_range(1),obj.gamma_range(2));
            % 状态更新
            obj.xr = obj.xr + obj.delta_t * obj.Vg * cos(obj.chi) * cos(obj.gamma);
            obj.yr = obj.yr + obj.delta_t * obj.Vg * sin(obj.chi) * cos(obj.gamma);
            obj.zr = obj.zr + obj.delta_t * obj.Vg * sin(obj.gamma);
            % 更新经纬度
            obj.current_lng = obj.init_lng + obj.xr / obj.lon_scale;
            obj.current_lat = obj.init_lat + obj.yr / obj.lat_scale;
            obj.current_hei = obj.zr;
            % 更新轨迹长度
            obj.traj_length = obj.traj_length + obj.delta_t * obj.Vg;
        end
        %% 得到当前点的eta_lon和eta_lat
        function [eta_lat,eta_lon] = get_eta_lon_lat(obj,x_p,y_p,z_p)
            % 需要跟踪的path_x,path_y,path_z
            x_r = obj.xr; y_r = obj.yr; z_r = obj.zr;
            gamma_r = obj.gamma; chi_r = obj.chi; 
            % 计算eta_lat和eta_lon
            eta_lat = atan2(y_p - y_r,x_p - x_r) - chi_r;
            eta_lon = atan2(z_p - z_r,sqrt((x_p - x_r)^2 + (y_p - y_r)^2)) - gamma_r;
        end
        %% 饱和函数
        function x_sat = clip(obj,x,xmin,xmax)
            if x > xmax
                x_sat = xmax;
            elseif x < xmin
                x_sat = xmin;
            else
                x_sat = x;
            end
        end
        %% 将当前路径序列从经纬度高转化成 x,y,z
        function [path_x,path_y,path_z] = transfer_lng_lat2xyz(obj,path_lng,path_lat,path_hei)
            path_x = (path_lng - obj.init_lng) * obj.lon_scale;
            path_y = (path_lat - obj.init_lat) * obj.lat_scale;
            path_z = path_hei;
        end
        %% 可视化
        function  draw_all(obj,ax)
            % 清楚当前的
            color_array = {'b','#A2142F','#4DBEEE','#77AC30',...
                            '#7E2F8E','#EDB120','#D95319','#0072BD'};
            num_color = length(color_array);
            for k = 1:obj.Paths.length
                path_data = obj.Paths.data{k};  % 路径编号
                task_index = obj.Tasks.data{k}; % 任务编号
                line(ax,path_data(:,1),...
                      path_data(:,2),...
                      path_data(:,3),...
                      'Color',color_array{1 + mod(task_index,num_color)},...
                      'Linestyle','-','Linewidth',2);
%                 text(ax,path_data(1,1),path_data(1,2),path_data(1,3),...
%                        num2str(task_index));
                % 开始一段路径起始点标记ro
                if k == 1
                  plot3(path_data(1,1),path_data(1,2),path_data(1,3)+10,...
                                             'ro','LineWidth',2);
                end
                % 最后一段路径结束点标记黑+
                if k == obj.Paths.length
                    plot3(path_data(end,1),path_data(end,2),path_data(end,3),...
                                                         'k+','LineWidth',2);
                end
            end
        end
        %% 保存path数据
        function save_path(obj,k)
            % 保存第k个路径
            path_data = obj.Paths.data{k};
            path_lng = path_data(:,1);
            path_lat = path_data(:,2);
            path_hei = path_data(:,3);
            ucav1 = obj;
            save(['path',num2str(k),'.mat'],'ucav1','path_lng','path_lat','path_hei');
        end
        %% 克隆当前对象
        function obj_new = clone(obj)
            obj_new = UCAV(); % 克隆一个自己
            % 对属性一一赋值
            obj_new.lon_scale = obj.lon_scale;
            obj_new.lat_scale = obj.lat_scale;
            obj_new.alive = obj.alive;
            obj_new.vel = obj.vel;
            obj_new.Vg = obj.Vg;
            obj_new.delta_t = obj.delta_t;
            obj_new.MaxAngle = obj.MaxAngle;
            obj_new.current_task = obj.current_task;
            obj_new.init_lng = obj.init_lng ;
            obj_new.init_lat = obj.init_lat;
            obj_new.dist = obj.dist;
            obj_new.time = obj.time;
            obj_new.cost = obj.cost;
            obj_new.Tasks = obj.Tasks.clone();
            obj_new.Paths = obj.Paths.clone();
            obj_new.task_star = obj.task_star.clone();
            obj_new.path_star = obj.path_star.clone();
            obj_new.dist_star = obj.dist_star;
            obj_new.time_star = obj.time_star;
            obj_new.cost_star = obj.cost_star;
        end
    end
end