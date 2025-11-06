function parameters()
    dt=1;%单位：s,原文没有提到时间间隔
    ts=90;%单位：s,飞行器终止制导时间
    g0=9.8;%重力常数，单位：m/s^2
    J2=0.00108263;%纬向系数
    R0=6356800;%地区半径，单位：m
    Cd_m=0.6;
    Cl_m=0.3;
    mass_m=32677;%地球质量
    alpha_max=0.1;%单位：弧度
    beta_max=0.1;%单位：弧度
    S_m=10.75%单位：m^2,认为火箭的直径为3.7m
    omega_e=7.292e-5;%地球自转角速度
    %Deviation
    rho_bias=0.2;%20%
    Cd_bias=0.2;
    Cl_bias=0.2;
    mass_bias=10;%+-10kg
    V_bias=50;%+-50m/s
    X_bias=5000;%+-5000

    %fitting
    KD=1.0;%真实偏差系数
    KL=1.0;
    KD_m=1.0;%自适应偏差系数
    KL_m=1.0;
    filePath = fullfile(pwd, 'para.mat');
    %PCG
    terminal_X_error=50;
    terminal_Y_error=200;


    save(filePath, 'Cd_m', 'Cl_m', 'mass_m', 'alpha_max', 'beta_max',...
         'KD','KL','KD_m','KL_m','g0','S_m','J2','R0','omega_e','dt',...
        'rho_bias', 'Cd_bias', 'Cl_bias', 'mass_bias', 'V_bias', 'X_bias',...
         'ts');
end
function rho_m=rho_calculate(r)
    %参数r:火箭到地心的纵向距离，单位：m
    rho_m=1.225*exp(-r/7717);
end
