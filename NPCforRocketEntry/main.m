parameters();
fprintf('初始化仿真参数...\n');
params=load('params.mat');
    
% 蒙特卡洛仿真设置
num_monte_carlo = 1000;  % 蒙特卡洛仿真次数
recovery_type = 'launch_site';  % 'launch_site' 或 'ship'

% 存储所有仿真结果
all_results = struct();
% 主蒙特卡洛循环
for idx = 1:num_monte_carlo
    % 生成随机偏差
    deviations = generate_random_deviations();
        
    % 单次闭环仿真
    [state_history,adaptive_history,performance] = ...
        single_closed_loop_simulation(params, deviations, recovery_type);
        
    % 存储结果
    all_results(idx).deviations = deviations;
    all_results(idx).state_history= state_history;
    all_results(idx).adaptive_history = adaptive_history;
    all_results(idx).performance=performance;
end
plot_all();