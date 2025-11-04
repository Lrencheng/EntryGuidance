function parameters()
    Cd_m=0.6;
    Cl_m=0.3;
    mass_m=32677;
    alpha_max=0.1;%单位：弧度
    beta_max=0.1;%单位：弧度

    %Deviation
    rho_bias=0.2;%20%
    Cd_bias=0.2;
    Cl_bias=0.2;
    mass_bias=10;%+-10kg
    V_bias=50;%+-50m/s
    X_bias=5000;%+-5000

    filePath = fullfile(pwd, 'para.mat');
    save(filePath, 'Cd_m', 'Cl_m', 'mass_m', 'alpha_max', 'beta_max', ...
        'rho_bias', 'Cd_bias', 'Cl_bias', 'mass_bias', 'V_bias', 'X_bias');
end
function rho_m=rho_calculate(r)
    %参数r:火箭到地心的纵向距离，单位：m
    rho_m=1.225*exp(-r/7717);
end
