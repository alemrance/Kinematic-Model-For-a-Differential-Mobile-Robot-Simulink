r   = 0.045;       % wheel radius [m]
L   = 0.2194;      % distance between wheels [m]
x0 = [0; 0; 0]      % initial pose [x; y; thta;]
Ts = 0.01        % sample time
% Mutlak konum sensörü profili
% 'uwb' | 'rtk' | 'lidar_loc' | 'demo'
posSensor = 'demo';
switch posSensor
    case 'uwb',       sig_pos = 0.10; Ts_pos = 0.1;
    case 'rtk',       sig_pos = 0.02; Ts_pos = 0.1;
    case 'lidar_loc', sig_pos = 0.03; Ts_pos = 0.1;
    case 'demo',      sig_pos = 0.10; Ts_pos = 1;    % eğitim için, kayma net görünsün
end