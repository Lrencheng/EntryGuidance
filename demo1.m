function closed_loop_simulation()
    % 火箭垂直回收制导系统闭环仿真框架
    % 集成自适应气动力拟合和预测校正制导
    
    clear; clc; close all;
    
    %% 1. 仿真参数设置
    fprintf('初始化仿真参数...\n');
    params = initialize_simulation_parameters();
    
    %% 2. 蒙特卡洛仿真设置
    num_monte_carlo = 1000;  % 蒙特卡洛仿真次数
    recovery_type = 'launch_site';  % 'launch_site' 或 'ship'
    
    % 存储所有仿真结果
    all_results = struct();
    
    %% 3. 主蒙特卡洛循环
    for mc_idx = 1:num_monte_carlo
        fprintf('运行蒙特卡洛仿真 %d/%d...\n', mc_idx, num_monte_carlo);
        
        % 生成随机偏差
        deviations = generate_random_deviations();
        
        % 单次闭环仿真
        [trajectory, guidance_performance, adaptive_history] = ...
            single_closed_loop_simulation(params, deviations, recovery_type);
        
        % 存储结果
        all_results(mc_idx).deviations = deviations;
        all_results(mc_idx).trajectory = trajectory;
        all_results(mc_idx).performance = guidance_performance;
        all_results(mc_idx).adaptive_history = adaptive_history;
    end
    
    %% 4. 结果分析与可视化
    analyze_and_plot_results(all_results, recovery_type);
    
    fprintf('闭环仿真完成！\n');
end

%% 子函数定义

function params = initialize_simulation_parameters()
    % 初始化仿真参数
    
    params = struct();
    
    % 物理常数
    params.R0 = 6378137;          % 地球半径 [m]
    params.mu = 3.986004418e14;   % 地球引力常数
    params.J2 = 0.00108263;       % J2系数
    params.omega_e = 7.292115e-5; % 地球自转角速度 [rad/s]
    params.g0 = 9.80665;          % 海平面重力加速度 [m/s²]
    
    % 火箭参数（绑定值 - 论文表2）
    params.mass_m = 32677;        % 质量 [kg]
    params.Cd_m = 0.6;            % 阻力系数
    params.Cl_m = 0.3;            % 升力系数
    params.S_m = 28.3;            % 参考面积 [m²] (基于Falcon 9直径)
    params.alpha_max = 0.1;       % 最大攻角 [rad]
    params.beta_max = 0.1;        % 最大侧滑角 [rad]
    
    % 仿真参数
    params.dt = 0.1;              % 积分时间步长 [s]
    params.control_update_rate = 1.0; % 控制更新频率 [s]
    params.adaptive_update_rate = 0.5; % 自适应更新频率 [s]
    
    % 终止条件
    params.termination_altitude = 40000;  % 终止高度 [m] (40km)
    params.min_velocity = 100;            % 最小速度 [m/s]
    
    % 目标位置（发射坐标系）
    params.target_position = [0, 0, 0];   % [X, Y, Z] [m]
    
    % 自适应参数初始值
    params.KD_m_initial = 1.0;
    params.KL_m_initial = 1.0;
    
    % 制导参数
    params.max_guidance_iterations = 3;   % 最大制导迭代次数
    params.position_tolerance = 10;       % 位置容差 [m]
end

function deviations = generate_random_deviations()
    % 生成随机偏差（论文表1）
    
    deviations = struct();
    
    % 大气密度偏差 ±20%
    deviations.rho_scale = 1 + (rand() - 0.5) * 0.4;
    
    % 气动系数偏差 ±20%
    deviations.Cd_scale = 1 + (rand() - 0.5) * 0.4;
    deviations.Cl_scale = 1 + (rand() - 0.5) * 0.4;
    
    % 质量偏差 ±10kg
    deviations.mass_delta = (rand() - 0.5) * 20;
    
    % 初始速度偏差 ±50m/s
    deviations.V_delta = (rand() - 0.5) * 100;
    
    % 初始位置偏差 ±5000m (随机方向)
    pos_magnitude = rand() * 5000;
    pos_angle = rand() * 2 * pi;
    deviations.X_delta = pos_magnitude * cos(pos_angle);
    deviations.Z_delta = pos_magnitude * sin(pos_angle);
    deviations.Y_delta = 0;  % 假设垂直平面内的偏差
end

function [trajectory, performance, adaptive_history] = ...
    single_closed_loop_simulation(params, deviations, recovery_type)
    % 单次闭环仿真
    
    %% 初始化
    % 初始状态（考虑偏差）
    initial_state = initialize_state(params, deviations, recovery_type);
    
    % 控制变量初始化
    control = initialize_control(params);
    
    % 自适应参数初始化
    adaptive_params = initialize_adaptive_params(params);
    
    % 制导参数初始化
    guidance_params = initialize_guidance_params(params);
    
    % 预分配存储
    max_steps = 10000;
    trajectory = preallocate_trajectory(max_steps);
    adaptive_history = preallocate_adaptive_history(max_steps);
    
    % 状态缓冲区（用于自适应拟合）
    state_buffer = initial_state;
    time_buffer = 0;
    
    % 当前状态
    current_state = initial_state;
    current_time = 0;
    step = 1;
    
    %% 主仿真循环
    while ~check_termination_condition(current_state, params)
        
        % 存储当前状态
        trajectory = store_trajectory_data(trajectory, step, current_state, control, current_time);
        
        % 更新状态缓冲区
        state_buffer(end+1, :) = current_state;
        time_buffer(end+1) = current_time;
        
        % 保持缓冲区大小
        if size(state_buffer, 1) > 10
            state_buffer = state_buffer(end-9:end, :);
            time_buffer = time_buffer(end-9:end);
        end
        
        %% 1. 自适应气动力拟合（定期执行）
        if mod(current_time, params.adaptive_update_rate) < params.dt && step > 1
            if size(state_buffer, 1) >= 2
                [adaptive_params, adaptive_info] = update_adaptive_parameters(...
                    state_buffer, time_buffer, control, adaptive_params, params);
                
                % 存储自适应历史
                adaptive_history = store_adaptive_history(...
                    adaptive_history, step, adaptive_params, adaptive_info);
            end
        end
        
        %% 2. 预测校正制导（定期执行）
        if mod(current_time, params.control_update_rate) < params.dt
            [control, guidance_info] = predictor_corrector_guidance(...
                current_state, control, adaptive_params, guidance_params, params);
            
            % 存储制导信息
            trajectory.guidance_info(step) = guidance_info;
        end
        
        %% 3. 动力学积分
        next_state = integrate_dynamics(...
            current_state, control, adaptive_params, params, params.dt);
        
        %% 4. 更新状态和时间
        current_state = next_state;
        current_time = current_time + params.dt;
        step = step + 1;
        
        % 检查最大步数
        if step > max_steps
            warning('达到最大仿真步数');
            break;
        end
    end
    
    %% 最终状态处理
    trajectory = store_trajectory_data(trajectory, step, current_state, control, current_time);
    
    % 计算制导性能
    performance = calculate_guidance_performance(trajectory, params);
    
    % 截断预分配的数组
    trajectory = truncate_trajectory(trajectory, step);
    adaptive_history = truncate_adaptive_history(adaptive_history, step);
end

function state = initialize_state(params, deviations, recovery_type)
    % 初始化状态向量
    
    % 根据回收类型设置不同的初始条件
    if strcmp(recovery_type, 'launch_site')
        % 返回发射场 - 较低初始速度
        initial_altitude = 80000;  % [m]
        initial_velocity = 2500;   % [m/s]
    else
        % 海上回收 - 较高初始速度
        initial_altitude = 60000;  % [m]
        initial_velocity = 3000;   % [m/s]
    end
    
    % 应用偏差
    initial_velocity = initial_velocity + deviations.V_delta;
    
    % 状态向量: [r, theta, phi, V, gamma, psi]
    state = zeros(6, 1);
    state(1) = params.R0 + initial_altitude;  % 地心距
    state(2) = 0;                             % 经度 [rad]
    state(3) = 0;                             % 纬度 [rad]
    state(4) = initial_velocity;              % 速度 [m/s]
    state(5) = -10 * pi/180;                  % 航迹角 [rad] (俯冲)
    state(6) = 0;                             % 航向角 [rad]
    
    % 应用位置偏差（转换为球坐标偏差）
    % 这里简化处理，实际需要更复杂的坐标转换
end

function control = initialize_control(params)
    % 初始化控制变量
    
    control = struct();
    control.alpha_k = 0.05;   % 攻角控制系数
    control.beta_k = 0.05;    % 侧滑角控制系数
    control.alpha = 0;        % 当前攻角 [rad]
    control.beta = 0;         % 当前侧滑角 [rad]
end

function adaptive_params = initialize_adaptive_params(params)
    % 初始化自适应参数
    
    adaptive_params = struct();
    adaptive_params.KD_m = params.KD_m_initial;
    adaptive_params.KL_m = params.KL_m_initial;
    adaptive_params.KD_history = [];
    adaptive_params.KL_history = [];
    adaptive_params.convergence_history = [];
    adaptive_params.residual_history = [];
end

function guidance_params = initialize_guidance_params(params)
    % 初始化制导参数
    
    guidance_params = struct();
    guidance_params.iteration_count = 0;
    guidance_params.position_error_history = [];
    guidance_params.control_history = [];
end

function [adaptive_params, adaptive_info] = update_adaptive_parameters(...
    state_buffer, time_buffer, control, adaptive_params, params)
    % 更新自适应气动力参数
    
    adaptive_info = struct();
    adaptive_info.success = false;
    adaptive_info.residual = inf;
    
    % 使用最近的两个状态点
    if size(state_buffer, 1) < 2
        return;
    end
    
    prev_state = state_buffer(end-1, :);
    curr_state = state_buffer(end, :);
    dt = time_buffer(end) - time_buffer(end-1);
    
    if dt <= 0
        return;
    end
    
    try
        % 调用修正后的ADF函数
        [KD_updated, KL_updated, converged, residual] = ADF_calculate_enhanced(...
            params, control, prev_state, curr_state, dt, adaptive_params);
        
        if converged
            % 应用滤波更新
            alpha = 0.7;  % 滤波系数
            adaptive_params.KD_m = alpha * adaptive_params.KD_m + (1-alpha) * KD_updated;
            adaptive_params.KL_m = alpha * adaptive_params.KL_m + (1-alpha) * KL_updated;
            
            % 限制范围
            adaptive_params.KD_m = max(0.3, min(2.0, adaptive_params.KD_m));
            adaptive_params.KL_m = max(0.3, min(2.0, adaptive_params.KL_m));
            
            adaptive_info.success = true;
            adaptive_info.residual = residual;
        end
        
    catch ME
        warning('自适应参数更新失败: %s', ME.message);
    end
    
    % 记录历史
    adaptive_params.KD_history(end+1) = adaptive_params.KD_m;
    adaptive_params.KL_history(end+1) = adaptive_params.KL_m;
    adaptive_params.convergence_history(end+1) = adaptive_info.success;
    adaptive_params.residual_history(end+1) = adaptive_info.residual;
end

function [control, guidance_info] = predictor_corrector_guidance(...
    current_state, control, adaptive_params, guidance_params, params)
    % 预测校正制导算法
    
    guidance_info = struct();
    guidance_info.iteration_count = 0;
    guidance_info.position_error = inf;
    guidance_info.converged = false;
    
    % 简化实现 - 实际需要完整的预测校正算法
    % 这里使用基于位置误差的简单控制律
    
    % 计算当前位置（转换为笛卡尔坐标）
    [current_pos, target_pos] = calculate_positions(current_state, params);
    
    % 计算位置误差
    pos_error = current_pos - target_pos;
    horizontal_error = norm(pos_error(1:2));
    
    % 简单的比例控制
    k_alpha = 0.001;  % 攻角控制增益
    k_beta = 0.001;   % 侧滑角控制增益
    
    % 更新控制系数（简化）
    control.alpha_k = control.alpha_k - k_alpha * pos_error(1);
    control.beta_k = control.beta_k - k_beta * pos_error(2);
    
    % 限制控制系数
    control.alpha_k = max(0.001, min(0.1, control.alpha_k));
    control.beta_k = max(0.001, min(0.1, control.beta_k));
    
    % 计算当前控制角度
    t = guidance_params.iteration_count * params.control_update_rate;
    t_s = 200;  % 假设的停止制导时间
    
    control.alpha = -control.alpha_k * (t - t_s);
    control.beta = -control.beta_k * (t - t_s);
    
    % 限制控制角度
    control.alpha = max(-params.alpha_max, min(params.alpha_max, control.alpha));
    control.beta = max(-params.beta_max, min(params.beta_max, control.beta));
    
    guidance_info.position_error = horizontal_error;
    guidance_info.iteration_count = guidance_params.iteration_count + 1;
    guidance_info.converged = (horizontal_error < params.position_tolerance);
    
    % 更新制导参数
    guidance_params.iteration_count = guidance_info.iteration_count;
    guidance_params.position_error_history(end+1) = horizontal_error;
    guidance_params.control_history(end+1) = [control.alpha_k, control.beta_k];
end

function next_state = integrate_dynamics(current_state, control, adaptive_params, params, dt)
    % 动力学积分（使用自适应参数）
    
    % 使用四阶龙格-库塔方法
    k1 = rocket_dynamics(current_state, control, adaptive_params, params);
    k2 = rocket_dynamics(current_state + 0.5*dt*k1, control, adaptive_params, params);
    k3 = rocket_dynamics(current_state + 0.5*dt*k2, control, adaptive_params, params);
    k4 = rocket_dynamics(current_state + dt*k3, control, adaptive_params, params);
    
    next_state = current_state + (dt/6) * (k1 + 2*k2 + 2*k3 + k4);
end

function state_dot = rocket_dynamics(state, control, adaptive_params, params)
    % 火箭动力学方程（使用自适应参数）
    
    % 计算气动力（使用自适应系数）
    [X_m, Y_m, Z_m] = aerodynamic_calculate_adaptive(params, state, control, adaptive_params);
    
    % 计算重力和科里奥利力
    [g_r, g_phi] = gravityVec_calculate(params, state);
    [C_v, C_gamma, C_psi] = CoriolisAcc_calculate(params, state);
    
    % 提取状态变量
    r = state(1); theta = state(2); phi = state(3);
    V = state(4); gamma = state(5); psi = state(6);
    
    % 动力学方程（论文公式1-6）
    state_dot = zeros(6,1);
    state_dot(1) = V * sin(gamma);
    state_dot(2) = V * cos(gamma) * sin(psi) / (r * cos(phi));
    state_dot(3) = V * cos(gamma) * cos(psi) / r;
    state_dot(4) = -X_m - g_r * sin(gamma) - g_phi * cos(gamma) * cos(psi) + C_v;
    state_dot(5) = Y_m/V + (V^2/r - g_r) * cos(gamma)/V + g_phi * sin(gamma) * cos(psi) + C_gamma;
    state_dot(6) = -Z_m/(V * cos(gamma)) + V/r * cos(gamma) * sin(psi) * tan(phi) + ...
                   g_phi * sin(psi) * sin(gamma)/V + C_psi;
end

function terminate = check_termination_condition(state, params)
    % 检查终止条件
    
    altitude = state(1) - params.R0;
    velocity = state(4);
    
    terminate = (altitude <= params.termination_altitude) || ...
                (velocity <= params.min_velocity);
end

function performance = calculate_guidance_performance(trajectory, params)
    % 计算制导性能
    
    performance = struct();
    
    % 最终位置误差
    final_pos = trajectory.position(end, :);
    target_pos = params.target_position;
    pos_error = final_pos - target_pos;
    
    performance.final_position_error = norm(pos_error);
    performance.final_altitude_error = pos_error(3);
    performance.final_horizontal_error = norm(pos_error(1:2));
    
    % 统计信息
    performance.max_position_error = max(trajectory.position_error);
    performance.mean_position_error = mean(trajectory.position_error);
    
    % 燃料消耗估计（简化）
    performance.fuel_consumption = 0;  % 实际需要根据发动机模型计算
end

function analyze_and_plot_results(all_results, recovery_type)
    % 结果分析和绘图
    
    fprintf('\n=== 蒙特卡洛仿真结果分析 ===\n');
    fprintf('回收类型: %s\n', recovery_type);
    fprintf('仿真次数: %d\n', length(all_results));
    
    % 提取终端误差
    terminal_errors = zeros(length(all_results), 3); % [X, Z, 总误差]
    
    for i = 1:length(all_results)
        perf = all_results(i).performance;
        terminal_errors(i, 1) = perf.final_position_error;
        terminal_errors(i, 2) = perf.final_horizontal_error;
        terminal_errors(i, 3) = perf.final_altitude_error;
    end
    
    % 计算统计量（对应论文表3）
    fprintf('\n终端位置误差统计:\n');
    fprintf('X方向 - 均值: %.2f m, 标准差: %.2f m\n', ...
        mean(terminal_errors(:,1)), std(terminal_errors(:,1)));
    fprintf('Z方向 - 均值: %.2f m, 标准差: %.2f m\n', ...
        mean(terminal_errors(:,2)), std(terminal_errors(:,2)));
    fprintf('总误差 - 均值: %.2f m, 标准差: %.2f m\n', ...
        mean(terminal_errors(:,3)), std(terminal_errors(:,3)));
    
    % 绘图
    plot_monte_carlo_results(all_results, recovery_type);
    plot_adaptive_parameters(all_results);
    plot_trajectory_examples(all_results);
end

function plot_monte_carlo_results(all_results, recovery_type)
    % 绘制蒙特卡洛结果
    
    figure('Position', [100, 100, 1200, 800]);
    
    % 子图1: 终端位置误差分布
    subplot(2,2,1);
    terminal_errors = zeros(length(all_results), 2);
    for i = 1:length(all_results)
        terminal_errors(i,1) = all_results(i).performance.final_horizontal_error;
        terminal_errors(i,2) = all_results(i).performance.final_altitude_error;
    end
    
    scatter(terminal_errors(:,1), terminal_errors(:,2), 20, 'filled', 'b');
    xlabel('水平位置误差 (m)');
    ylabel('高度误差 (m)');
    title('终端位置误差分布');
    grid on;
    axis equal;
    
    % 子图2: 位置误差随时间变化
    subplot(2,2,2);
    hold on;
    for i = 1:min(10, length(all_results))  % 绘制前10次仿真
        plot(all_results(i).trajectory.time, all_results(i).trajectory.position_error);
    end
    xlabel('时间 (s)');
    ylabel('位置误差 (m)');
    title('位置误差收敛过程');
    grid on;
    
    % 子图3: 控制变量变化
    subplot(2,2,3);
    if length(all_results) >= 1
        plot(all_results(1).trajectory.time, all_results(1).trajectory.alpha, 'b-', 'LineWidth', 2);
        hold on;
        plot(all_results(1).trajectory.time, all_results(1).trajectory.beta, 'r-', 'LineWidth', 2);
        legend('攻角', '侧滑角');
        xlabel('时间 (s)');
        ylabel('控制角度 (rad)');
        title('控制变量变化');
        grid on;
    end
    
    % 子图4: 自适应系数收敛
    subplot(2,2,4);
    if length(all_results) >= 1
        adaptive_history = all_results(1).adaptive_history;
        plot(adaptive_history.time, adaptive_history.KD_m, 'b-', 'LineWidth', 2);
        hold on;
        plot(adaptive_history.time, adaptive_history.KL_m, 'r-', 'LineWidth', 2);
        legend('K_D', 'K_L');
        xlabel('时间 (s)');
        ylabel('自适应系数');
        title('自适应系数收敛过程');
        grid on;
    end
    
    sgtitle(sprintf('火箭垂直回收制导 - %s回收模式', recovery_type));
end

% 辅助函数（需要根据实际情况实现）
function trajectory = preallocate_trajectory(max_steps)
    % 预分配轨迹存储
    trajectory = struct();
    trajectory.time = zeros(max_steps, 1);
    trajectory.position = zeros(max_steps, 3);
    trajectory.velocity = zeros(max_steps, 3);
    trajectory.alpha = zeros(max_steps, 1);
    trajectory.beta = zeros(max_steps, 1);
    trajectory.position_error = zeros(max_steps, 1);
    trajectory.guidance_info = cell(max_steps, 1);
end

function adaptive_history = preallocate_adaptive_history(max_steps)
    % 预分配自适应历史存储
    adaptive_history = struct();
    adaptive_history.time = zeros(max_steps, 1);
    adaptive_history.KD_m = zeros(max_steps, 1);
    adaptive_history.KL_m = zeros(max_steps, 1);
    adaptive_history.residual = zeros(max_steps, 1);
    adaptive_history.converged = false(max_steps, 1);
end

function trajectory = store_trajectory_data(trajectory, step, state, control, time)
    % 存储轨迹数据
    trajectory.time(step) = time;
    trajectory.position(step, :) = state(1:3);
    trajectory.velocity(step, :) = state(4:6);
    trajectory.alpha(step) = control.alpha;
    trajectory.beta(step) = control.beta;
end

function adaptive_history = store_adaptive_history(adaptive_history, step, adaptive_params, adaptive_info)
    % 存储自适应历史
    adaptive_history.time(step) = step;  % 简化，实际应该用仿真时间
    adaptive_history.KD_m(step) = adaptive_params.KD_m;
    adaptive_history.KL_m(step) = adaptive_params.KL_m;
    adaptive_history.residual(step) = adaptive_info.residual;
    adaptive_history.converged(step) = adaptive_info.success;
end

function trajectory = truncate_trajectory(trajectory, actual_steps)
    % 截断轨迹数组
    fields = fieldnames(trajectory);
    for i = 1:length(fields)
        field = fields{i};
        if isnumeric(trajectory.(field))
            trajectory.(field) = trajectory.(field)(1:actual_steps, :);
        elseif iscell(trajectory.(field))
            trajectory.(field) = trajectory.(field)(1:actual_steps);
        end
    end
end

function adaptive_history = truncate_adaptive_history(adaptive_history, actual_steps)
    % 截断自适应历史数组
    fields = fieldnames(adaptive_history);
    for i = 1:length(fields)
        field = fields{i};
        adaptive_history.(field) = adaptive_history.(field)(1:actual_steps, :);
    end
end

% 运行仿真
closed_loop_simulation();