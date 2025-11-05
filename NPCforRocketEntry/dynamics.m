function state_dot=rocketdynamics(params,state,control)
    %{
    state状态变量：
    1. r :火箭到地心的距离,m
    2. theta :火箭的经度，longitude,rad
    3. phi :火箭的纬度，geocentric latitude,rad
    4. V :火箭速度,m/s
    5. gamma:火箭航迹角，geocentric latitude,rad
    6. Psi ：火箭航向角，heading angle，rad
    %}
    %{
    control控制变量：
    1. alpha :倾侧角
    2. beta :侧滑角
    %}
    r=state(1);
    theta=state(2);
    phi=state(3);
    V=state(4);
    gamma=state(5);
    Psi=state(6);

    theta_dot=V*cos(gamma)*sin(Psi)/(r*cos(phi));
    phi_dot=V*cos(gamma)*cos(Psi)/r;
    
end
function [X_m,Y_m,Z_m]=aerodynamic_calculate(params,state)
    r=state(1);V=state(4);
    rho=rho_calculate(r);
    q=0.5*rho*V^2;
    X_m=params.KD_m*params.Cd_m*q*params.Sm;
end