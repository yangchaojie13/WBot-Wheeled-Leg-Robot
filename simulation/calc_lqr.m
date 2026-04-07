%% 3D 轮腿机器人 LQR 控制器设计脚本 (V1 - 修正版)
% 基于论文: Dynamic Modeling of a Two-wheeled Inverted Pendulum...
% ===============================================================
clear; clc; close all;

%% 1. 物理参数定义 (来自 SolidWorks 测量)
% -----------------------------------------------------------
% [1.1] 几何参数
r = 0.01830;    
d = 0.07902;    
% [1.2] 摆体参数 (Body) 
m_B = 0.280;     
l   = 0.03878;    
I1  = 0.000358253;  
I2  = 0.000215645;  
I3  = 0.000258705;  
% [1.3] 轮子参数 (Wheel)
m_W = 0.036;        
J   = 0.000005943;    
K   = 0.000004057;

% [1.4] 环境与电机常数
g   = 9.81;         % [m/s^2] 重力加速度
b   = 0.000;        % [N*m*s] 粘滞摩擦系数 (仿真初期设为0)

% [1.5] 控制周期
Ts  = 0.001;        % [s] 控制循环时间 (1kHz = 0.001s)

%% 2. 构建动力学矩阵 (基于论文 Appendix A)
% -----------------------------------------------------------
% 我们在平衡点 (theta=0, dot_q=0) 处计算矩阵数值

% 辅助变量 (theta = 0 时)
sin_0 = 0; 
cos_0 = 1;

% [2.1] 惯性矩阵 M (Mass Matrix)
% 对应论文公式: M(q) * q_ddot
a11 = m_B + 2*m_W + 2*J/(r^2);
a12 = m_B * l * cos_0;
a22 = I2 + m_B * l^2;
a33 = I3 + 2*K + (m_W + J/(r^2))*d^2/2; % theta=0时, sin^2项消失

M = [a11, a12, 0;
     a12, a22, 0;
     0,   0,   a33];

% [2.2] 阻尼矩阵 D (Damping Matrix)
% 对应论文公式: D * q_dot
d11 = 2*b/(r^2);
d12 = -2*b/r;
d22 = 2*b;
d33 = (d^2 / (2*r^2)) * b;

D_mat = [d11, d12, 0;
         d12, d22, 0;
         0,   0,   d33];

% [2.3] 重力刚度矩阵 G_jac (Linearized Gravity)
% 原重力向量 G(q) = [0; -m_B*g*l*sin(theta); 0]
% 线性化后 G(q) ~ G_jac * q
% 对 theta 求导 -> -m_B*g*l*cos(0) = -m_B*g*l
G_stiffness = [0, 0, 0;
               0, -m_B*g*l, 0;
               0, 0, 0];

% [2.4] 输入矩阵 B (Input Matrix)
B_mat = [1/r,       1/r;
         -1,        -1;
         -d/(2*r),  d/(2*r)];

%% 3. 建立线性化状态空间模型 (Continuous State-Space)
% -----------------------------------------------------------
% 状态变量 X_full = [dot_x, dot_theta, dot_psi, x, theta, psi]'
% 维度: 6状态 x 2输入

% 计算 A_c 和 B_c
% 动力学方程: M * q_ddot + D * q_dot + G_stiffness * q = B_mat * u
% => q_ddot = -M\D * q_dot - M\G_stiffness * q + M\B_mat * u

invM = inv(M);

Ac_dyn = -invM * D_mat;       % 阻尼部分
Ac_stiff = -invM * G_stiffness; % 重力/刚度部分
Bc_dyn = invM * B_mat;

% 组装完整的 Ac, Bc
%      [ q_dot ]   [ q ]
Ac = [ Ac_dyn,     Ac_stiff;
       eye(3),     zeros(3,3) ];

Bc = [ Bc_dyn;
       zeros(3,2) ];

%% 4. 模型降阶与离散化 (Model Reduction & Discretization)
% -----------------------------------------------------------
% 原始状态: [dx, dtheta, dpsi, x, theta, psi] (1-6)
% 我们不需要控制绝对位置 x (状态4) 和 绝对航向 psi (状态6)
% 仅保留: [dx, dtheta, dpsi, theta]
% 对应索引: 1, 2, 3, 5

keep_idx = [1, 2, 3, 5];
Ar = Ac(keep_idx, keep_idx);
Br = Bc(keep_idx, :);

% 离散化 (Continuous -> Discrete)
sys_c = ss(Ar, Br, eye(4), 0);
sys_d = c2d(sys_c, Ts, 'zoh'); % 零阶保持器

Ad = sys_d.A;
Bd = sys_d.B;

%% 5. 增加积分环节 (Integral Action)
% -----------------------------------------------------------
% 我们希望无静差地跟踪：速度(dx) 和 转向速率(dpsi)
% 增加两个积分状态: 
% e_dx  = sum(ref_dx - dx)
% e_psi = sum(ref_dpsi - dpsi)

% C_track 矩阵: 提取我们需要跟踪的状态 (dx 和 dpsi)
% dx 是第1个, dpsi 是第3个
C_track = [1, 0, 0, 0;
           0, 0, 1, 0];

% 增广 A 矩阵
% [ Ad,  0 ]
% [ -Cd*Ad, I ]  <-- 这里的负号取决于积分器的定义 e(k+1) = e(k) + (r - y)
A_aug = [ Ad, zeros(4,2);
          -C_track*Ad*Ts, eye(2) ]; % 注意乘以Ts，使积分项具有物理意义

% 增广 B 矩阵
% [ Bd ]
% [ -Cd*Bd ]
B_aug = [ Bd;
          -C_track*Bd*Ts ];

%% 6. LQR 计算 (Calculate Gain K)
% -----------------------------------------------------------
% 权重矩阵 Q (State Cost) 和 R (Input Cost)
% 状态顺序: [dx, dtheta, dpsi, theta, int_dx, int_dpsi]

% --- 调参区域 (Tuning Section) ---
q_dx     = 100;     % 速度误差权重
q_dtheta = 1;       % 角速度权重 (通常小一点)
q_dpsi   = 150;     % 转向速度权重
q_theta  = 800;     % 角度权重 (越大越想保持直立)
q_idx    = 5000;     % 速度积分权重 (越大消除稳态误差越快)
q_ipsi   = 100;      % 转向积分权重

r_torque = 500;     % 扭矩消耗权重 (越小电机越暴力，越大越省电/温和)

Q = diag([q_dx, q_dtheta, q_dpsi, q_theta, q_idx, q_ipsi]);
R = diag([r_torque, r_torque]); % 左轮和右轮权重一样

% 计算 K 矩阵
[K, S, e] = dlqr(A_aug, B_aug, Q, R);

%% 7. 输出结果 (Output)
% -----------------------------------------------------------
fprintf('\n================ LQR 计算成功 ================\n');
fprintf('状态反馈增益矩阵 K (2x6):\n');
disp(K);

fprintf('请将以下参数填入您的 C++ 代码 (PID部分):\n');
fprintf('--------------------------------------------\n');
fprintf('// 对应状态顺序: [dx, dtheta, dpsi, theta, i_dx, i_psi]\n');
fprintf('float K_speed       = %.4f;\n', K(1,1)); % 对应 dx
fprintf('float K_pitch_rate  = %.4f;\n', K(1,2)); % 对应 dtheta
fprintf('float K_yaw_rate    = %.4f;\n', K(1,3)); % 对应 dpsi
fprintf('float K_pitch       = %.4f;\n', K(1,4)); % 对应 theta
fprintf('float K_speed_int   = %.4f;\n', K(1,5)); % 对应 积分项
fprintf('float K_yaw_int     = %.4f;\n', K(1,6)); % 对应 积分项
fprintf('--------------------------------------------\n');