function [alpha_k,beta_k]=PredictCorrector_guidance(t,params,state,prev_control_coef)
    
    % 当前控制系数初始猜测
    %{
    control_coef结构体：
    1.alpha_k
    2.beta_k
    %}
    x0 = [prev_control_coef.alpha_k, prev_control_coef.beta_k];
    
    function error=minfun(x0)
        state_p=terminus_calculate(t,params,state,control_coef);
        theta_p=state_p(2);phi_p=state_p(3);
        error=theta_p^2+phi_p^2;
    end
        % 设置优化选项
    options = optimoptions('fmincon', ...
        'Algorithm', 'interior-point', ...
        'Display', 'off', ...  % 关闭输出以提升速度
        'MaxIterations', 10, ...
        'StepTolerance', 1e-4, ...
        'ConstraintTolerance', 1e-4, ...
        'OptimalityTolerance',1e-4);
    solution=fmincon(@minfun,x0,[],[],[],[],[],[],[],options);
    if solution<=0
        fprintf("PCG>>控制参数ak,bk求解失败\n");
        error("原因：%d\n",solution.exitflag);
    else
        [alpha_k,beta_k]=solution.x;
    end
end