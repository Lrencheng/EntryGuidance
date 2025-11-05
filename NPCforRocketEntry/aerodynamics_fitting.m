function [KD_m,KL_m]=ADF_calculate(params,control,prev_state,curr_state)
    %aerodynamics fittinf气动力自适应算法的实现
    %论文参考公式：公式(16),公式(17),公式(18)
    V_prev=prev_state(4);
    V_curr=curr_state(4);
    r_prev=prev_state(1);

    [G_V,G_gamma,G_Psi]=GravityAccVector_calculate(params,prev_state);

    rho=rho_calculate(r_prev);%计算推测气压值：含有误差
    q=0.5*rho*V_prev^2;
    base_x=params.Cd_m*q*params.S_m/params.mass_m;
    KD_m=(V_prev-V_curr+G_v)/base_x;%计算自适应阻力偏差系数KD_m，使用简单线性求解方法

    %使用数值解法解决KL_m问题
    
end
function [G_V,G_gamma,G_Psi]=GravityAccVector_calculate(params,prev_state)
    r=prev_state(1);
    phi=prev_state(3);
    V=prev_state(4);
    gamma=prev_state(5);
    Psi=prev_state(6);

    [g_r,g_phi]=gravityVec_calculate(params,prev_state);
    
    G_v=-g_r*sin(gamma)-g_phi*cos(gamma)*cos(Psi);
    G_gamma=(V^2/r-g_r)*cos(gamma)/V+g_phi*sin(gamma)*cos(Psi);
    G_Psi=V/r*cos(gamma)*sin(Psi)*tan(phi)+g_phi*sin(gamma)*cos(Psi);

end 