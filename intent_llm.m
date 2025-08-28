clc;
close all;
clear all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% % Example usage of intent_to_actions_gemini.m
% generate_with_gemini("Rotate servo 90 degrees, wait 2 seconds, then return to 0.");
% 
% function generate_with_gemini(prompt)
%     % Set your Gemini API Key (get it from AI Studio: https://aistudio.google.com/app/apikey)
%     apiKey = getenv("GEMINI_API_KEY");
%     if isempty(apiKey)
%         error("GEMINI_API_KEY environment variable not set. Run: setx GEMINI_API_KEY ""your_key_here"" ");
%     end
% 
%     % Gemini endpoint
%     url = "https://generativelanguage.googleapis.com/v1beta/models/gemini-1.5-flash:generateContent?key=" + apiKey;
% 
%     % Build request JSON
%     data = struct( ...
%         "contents", {struct( ...
%             "role","user", ...
%             "parts", {struct("text", prompt)} ...
%         )}, ...
%         "generationConfig", struct( ...
%             "response_mime_type","application/json" ...
%         ) ...
%     );
% 
%     % Convert to JSON string
%     body = jsonencode(data);
% 
%     % Web options
%     opts = weboptions( ...
%         "MediaType","application/json", ...
%         "Timeout",60 ...
%     );
% 
%     % Make the request
%     response = webwrite(url, body, opts);
% 
%     % Extract candidates
%     if isfield(response,"candidates") && ~isempty(response.candidates)
%         disp("---- Gemini Response ----");
%         disp(response.candidates(1).content.parts(1).text);
%     else
%         disp("No candidates returned.");
%     end
% end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


% % Example raw JSON text from Gemini
% rawText = '{"commands": [{"servo": {"angle": 90, "time": 2000}}, {"servo": {"angle": 0, "time": 0}}, {"servo": {"angle": 45, "time": 1000}}]}';
% 
% % Decode JSON into MATLAB struct
% cmds = jsondecode(rawText);
% 
% % Preallocate arrays
% n = numel(cmds.commands);
% angles = zeros(1,n);
% times  = zeros(1,n);
% 
% % Extract each command
% for k = 1:n
%     angles(k) = cmds.commands(k).servo.angle;
%     times(k)  = cmds.commands(k).servo.time;
% end
% 
% % Show result
% disp("Decoded Servo Commands:");
% table(times(:), angles(:), 'VariableNames', {'Time_ms','Angle_deg'})

% servo_tracking_discrete.m
% Discrete-time state-space servo tracking of JSON commands using LQR + Nbar.
% Copy this file and run in MATLAB.

clear; 
close all; 
clc;

%% === Example JSON (replace rawText with real Gemini output) ===
rawText = '{"commands": [{"servo": {"angle": 90, "time": 2000}}]}';

%% === Decode JSON and extract angles & durations ===
cmds = jsondecode(rawText);
nCmd = numel(cmds.commands);
angles_deg = zeros(1, nCmd);
dur_ms     = zeros(1, nCmd);

for k = 1:nCmd
    entry = cmds.commands(k).servo;
    if ~isfield(entry,'angle') || ~isfield(entry,'time')
        error('Each servo command must include angle and time fields.');
    end
    angles_deg(k) = double(entry.angle);
    dur_ms(k)     = double(entry.time);
end

% Interpret zero durations: replace 0 by a small hold so interpolation works
min_hold = 100; % ms (0.1 s)
dur_ms(dur_ms <= 0) = min_hold;

% Build cumulative waypoint times in seconds
waypoint_t = [0, cumsum(dur_ms)/1000]; % include t0 = 0
% Build waypoint angles: assume initial angle = 0 deg at t=0,
% then each command's angle is the target at the corresponding waypoint time.
waypoint_angles = [0, angles_deg];

% If user wants initial angle different, change waypoint_angles(1).

%% === Build dense discrete time vector and desired trajectory (linear interp) ===
dt = 0.01;              % sampling period (s) — discrete-time step
t_final = waypoint_t(end) + 1.0;  % one extra second to settle
t = 0:dt:t_final;
% piecewise-linear interpolation of desired angle
ref_deg = interp1(waypoint_t, waypoint_angles, t, 'linear', 'extrap');
ref_rad = deg2rad(ref_deg);   % convert to radians for simulation

%% === Continuous-time second-order servo model (radian units) ===
% We'll use a standard mass-damper inertia model:
% J*theta_dd + b*theta_dot = K_t * u
% state x = [theta; theta_dot], input u = torque command
J = 2.5e-4;     % kg*m^2 (tune for your virtual servo)
b = 5e-6;       % N*m*s/rad (damping)
K_t = 1e-3;     % torque per command unit (N*m per control unit)

% Continuous A,B,C,D (theta in radians)
A_c = [0 1; 0 -b/J];
B_c = [0; K_t/J];
C_c = [1 0];
D_c = 0;

% Discretize
sys_c = ss(A_c, B_c, C_c, D_c);
sys_d = c2d(sys_c, dt);
A_d = sys_d.A; B_d = sys_d.B; C_d = sys_d.C; D_d = sys_d.D;

%% === LQR design (discrete) ===
% Choose Q and R to tune performance (state weighting and control cost)
Q = diag([200, 1]);   % weight angle error heavily
R = 1e-4;             % penalize control effort (lower -> stronger actuation)
[K, ~, ~] = dlqr(A_d, B_d, Q, R);

% Compute steady-state feedforward gain Nbar for setpoint tracking:
% solve u_ss such that y_ss = r and x_ss = A_d*x_ss + B_d*u_ss
% x_ss = inv(I - A_d) * B_d * u_ss; r = C_d * x_ss => u_ss = r / (C_d * inv(I-A_d) * B_d)
I = eye(size(A_d));
invIA = inv(I - A_d);
den = C_d * (invIA * B_d);
if abs(den) < 1e-12
    warning('Denominator for Nbar is tiny; feedforward may be unstable. Setting Nbar=1.');
    Nbar = 1;
else
    Nbar = 1 / den;   % scalar because single-output
end

%% === Simulate closed-loop discrete-time dynamics ===
N = length(t);
x = zeros(2, N);    % state history [theta; theta_dot]
y = zeros(1, N);    % output (theta)
u_hist = zeros(1, N);% control input (command signal)

% initial state: start at ref(1) if you want to start at initial reference
x(:,1) = [ref_rad(1); 0];  

for k = 1:N-1
    % current state
    xk = x(:,k);
    % reference value at this time (scalar)
    r_k = ref_rad(k);
    % control law: u = -K*x + Nbar * r
    u = -K * xk + Nbar * r_k;
    % optional saturation (simulate actuator limits)
    u = max(min(u, 5), -5);  % clamp to [-5, 5] (tweakable)
    % state update (discrete)
    x(:,k+1) = A_d * xk + B_d * u;
    y(k) = C_d * xk;
    u_hist(k) = u;
end
% final outputs
 y(N) = C_d * x(:,N);
u_hist(N) = u_hist(N-1);

% convert to degrees for plotting
y_deg = rad2deg(y);
ref_deg_plot = rad2deg(ref_rad);

%% === Plots: reference vs tracked, and control input ===
figure('Name','Discrete Servo Tracking','Units','normalized','Position',[0.1 0.1 0.8 0.7]);

subplot(2,1,1);
plot(t, ref_deg_plot, 'r--','LineWidth',1.4); hold on;
plot(t, y_deg, 'b-','LineWidth',1.8);
xlabel('Time (s)'); ylabel('Angle (deg)');
legend('Desired','Tracked','Location','best');
title('Desired vs Tracked Angle (Discrete LQR + Nbar)');
grid on;

subplot(2,1,2);
plot(t, u_hist, 'k-','LineWidth',1.4);
xlabel('Time (s)'); ylabel('Control input (u)');
title('Control input (command)');
grid on;

%% === Animation: servo arm moving over time ===
figure('Name','Servo Animation','Units','normalized','Position',[0.05 0.05 0.35 0.4]);
L = 0.08;  % arm length (m)
base = [0,0];

% prepare axes
ax = gca;
axis equal;
axis([-0.12 0.12 -0.12 0.12]);
hold on; grid on;
title('Servo Arm (Discrete Simulation)');
hArm = plot([0, L], [0,0], 'b-', 'LineWidth', 4);
hPivot = plot(0,0,'ko','MarkerSize',8,'MarkerFaceColor','k');
hText = text(-0.11,0.11,'', 'FontSize', 10);

% animate at sampling rate (or slower for smoothness)
frameStep = 1;  % animate every sample; increase for faster playback
for k = 1:frameStep:N
    th = deg2rad(y_deg(k)); % already rad but convert from deg just to be safe
    xend = L * cos(th);
    yend = L * sin(th);
    set(hArm, 'XData', [0 xend], 'YData', [0 yend]);
    set(hText, 'String', sprintf('t = %.2f s  | Desired = %.1f°  | Tracked = %.1f°', ...
        t(k), ref_deg_plot(k), y_deg(k)));
    drawnow;
    pause(0.01);  % control animation speed; adjust if needed
end

%% === Performance metrics (optional) ===
err = ref_deg_plot - y_deg;
rmse = sqrt(mean(err.^2));
fprintf('Tracking RMSE = %.4f deg over %.2f s\n', rmse, t(end));
