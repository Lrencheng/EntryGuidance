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
    state_dot=zeros(6,1);
    r=state(1);
    theta=state(2);
    phi=state(3);
    V=state(4);
    gamma=state(5);
    Psi=state(6);

    [X_m,Y_m,Z_m]=aerodynamic_calculate(params,state,control);
    [g_r,g_phi]=gravityVec_calculate(params,state);
    [C_v,C_gamma,C_Psi]=CoriolisAcc_calculate(params,state);

    r_dot=V*sin(gamma);
    theta_dot=V*cos(gamma)*sin(Psi)/(r*cos(phi));
    phi_dot=V*cos(gamma)*cos(Psi)/r;
    V_dot=-X_m-g_r*sin(gamma)-g_phi*cos(gamma)*cos(Psi)+C_v;
    gamma_dot=Y_m/V+(V^2/r-g_r)*cos(gamma)/V+g_phi*sin(gamma)*cos(Psi)+C_gamma;
    Psi_dot=-Z_m/(V*cos(gamma))+V*cos(gamma)*sin(Psi)*tan(phi)/r+g_phi*sin(gamma)*cos(Psi)+C_Psi;

    state_dot(1)=r_dot;
    state_dot(2)=theta_dot;
    state_dot(3)=phi_dot;
    state_dot(4)=V_dot;
    state_dot(5)=gamma_dot;
    state_dot(6)=Psi_dot;
end
function [X_m,Y_m,Z_m]=aerodynamic_calculate(params,state,control)
    %计算fitting气动力
    r=state(1);V=state(4);
    rho=rho_calculate(r);
    q=0.5*rho*V^2;
    X_m=params.KD_m*params.Cd_m*q*params.S_m/params.mass_m;
    Y_m=params.KL_m*params.Cl_m*control.alpha*q*params.S_m/params.mass_m;
    Z_m=params.KL_m*params.Cl_m*control.beta*q*params.S_m/params.mass_m;
end
function [g_r,g_phi]=gravityVec_calculate(params,state)
    u=params.g0;
    r=state(1);
    phi=state(3);
    R0=params.R0;
    J2=params.J2;
    g_r=u*(1+J2*(R0/r)^2*(1.5-4.5*sin(phi)^2))/(r^2);
    g_phi=u*(J2*(R0/r)^2*(3*sin(phi)*cos(phi)));
end
function [C_v,C_gamma,C_Psi]=CoriolisAcc_calculate(params,state)
    omega_e=params.omega_e;
    r=state(1);
    phi=state(3);
    V=state(4);
    gamma=state(5);
    Psi=state(6);
    C_v=omega_e^2*r*(cos(phi)^2*sin(gamma)-cos(phi)*sin(phi)*cos(Psi)*cos(gamma));
    C_gamma=2*omega_e*sin(Psi)*cos(phi)+...
            omega_e^2*r*cos(phi)*(sin(phi)*cos(Psi)*sin(gamma)+cos(phi)*cos(gamma));
    C_Psi=2*omega_e*(sin(phi)-cos(Psi)*tan(gamma)*cos(phi))+...
          omega_e^2*r*cos(phi)*sin(phi)*sin(Psi)/(V*cos(gamma));
end