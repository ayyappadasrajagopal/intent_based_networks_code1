% ibn_servo_llm_demo.m
% Discrete-state-space NCS demo using a DC servo motor model:
%  - LQR controller (nominal + safe fallback)
%  - Kalman filter for state estimation (position measurement only)
%  - Mock LLM natural-language intent parser
%  - Intent feasibility check vs network capability
%  - Runtime intent changes demonstrating negotiation and enforced QoS
%  - Network-induced delay & packet loss on sensor->controller and controller->actuator
%  - Plots: reference vs plant output, estimation error, control command, and performance metrics
%
% Author: ChatGPT (for Ayyappadas) - 2025

clear; close all; clc;

%% ---------------- Servo motor continuous model ------------------------
% DC servo motor (position control) - simplified 2nd order model:
% theta_ddot = (1/J)*(K_t * i - b*theta_dot)  and electrical dynamics approximated
% We will use a standard second-order transfer: Theta(s)/Voltage(s) = K/(Js^2 + b s)
J = 0.01;   % rotor inertia
b = 0.1;    % viscous friction
K = 0.01;   % motor constant (torque per amp) * amplifier gain effectively
% For simplicity collapse electrical time constant -> use a 2nd order plant:
num = K;
den = [J b 0];        % note: pure integrator on torque -> position
G_cont = tf(num, den); % position output per input voltage

% Add slight damping and stiffness by adding small stiffness term to den[3] to avoid pure integrator issues
den = [J b 0.1];  % add small term to make controllable second-order
G_cont = tf(K, den);

% Sampling / control time
Ts = 0.01;  % 10 ms
sysd = c2d(G_cont, Ts);
Ad = sysd.A; Bd = sysd.B; Cd = sysd.C; Dd = sysd.D;

nx = size(Ad,1);
nu = size(Bd,2);
ny = size(Cd,1);

fprintf('Plant discrete A size: %dx%d\n', size(Ad,1), size(Ad,2));

%% ---------------- LQR design (nominal + safe fallback) ----------------
Q_nom = diag([200, 5]);  % penalize position error strongly
R_nom = 0.01;
Q_safe = diag([50, 1]);
R_safe = 0.1;

try
    K_nom = lqr(Ad, Bd, Q_nom, R_nom);
    K_safe = lqr(Ad, Bd, Q_safe, R_safe);
catch
    warning('lqr() not available: using heuristic gains.');
    K_nom = [30 5];
    K_safe = [8 1.5];
end

%% --------------- Kalman Filter design (process + measurement noise) ------------
% Assume we only measure position (first state).
% Define process and measurement noise covariances for KF (discrete)
W = 1e-4 * eye(nx);  % process noise covariance
V = 5e-4;            % measurement noise (position sensor)

% Build discrete-time Kalman gain via discrete-time estimation Riccati
% Use dlqe-like approach: compute steady-state filter gain L satisfying discrete Riccati
P = dlyap(Ad, W);    % initial approx for P
% iterative Riccati (small number of iterations)
for i=1:50
    P = Ad*P*Ad' - Ad*P*Cd'*inv(Cd*P*Cd' + V)*Cd*P*Ad' + W;
end
L_kf = P*Cd' / (Cd*P*Cd' + V);

%% ---------------- Network capability & achievable model ----------------
network.base_delay = 0.008;    % base one-way delay (s)
network.base_loss  = 5e-5;     % base packet loss prob
network.load = 0.3;

achievableDelay = @(net) net.base_delay + 0.04*net.load;   % s
achievableLoss  = @(net) net.base_loss + 1e-3*net.load;    % prob

%% ------------------ Simulation parameters ------------------------------
Tstop = 12;            % seconds
N = round(Tstop / Ts);
t = (0:N-1)*Ts;

% Reference: a step at 0.5s followed by another change at 6s
r = zeros(1,N);
r(t >= 0.5) = 0.5;   % initial step to 0.5 rad
r(t >= 6.0) = -0.3;  % change reference at 6s to show re-convergence

% Initial state
x = zeros(nx,1);
xhat = zeros(nx,1);   % estimator state
u = 0;

% Buffers for network delay (we will simulate variable intent changes that alter enforced QoS)
max_delay_steps = 50; % cap for buffer size
sensor_buffer = zeros(nx, max_delay_steps+5);
actuator_buffer = zeros(nu, max_delay_steps+5);

% Logs
xlog = zeros(nx, N);
xhatlog = zeros(nx, N);
ulog = zeros(nu, N);
rlog = r;
measlog = zeros(ny, N);
esterr_log = zeros(nx, N);

% RNG
rng(2);

%% ------------------ LLM + intent schedule ------------------------------
% We'll change intents at runtime to show how negotiation and enforcement affect performance.
% Define textual intents and the times they become active.
intent_schedule = {
    0.0, "Keep actuator feedback delay below 15 ms, ensure very high reliability for sensors, priority safety";
    4.0, "Relax delay to 40 ms but maintain high reliability; best effort for video";
    8.0, "Strict: delay < 12 ms, very high reliability, safety priority"
};
next_intent_idx = 1;
current_intent = parse_intent_nl(intent_schedule{next_intent_idx,2});

% initial feasibility negotiation
ach_delay = achievableDelay(network);
ach_loss  = achievableLoss(network);
[enforced, enforcement_note] = negotiate_intent(current_intent, ach_delay, ach_loss);
fprintf('t=0.0s Active Intent: %s\n -> enforced delay=%.3fs, loss=%.1e (%s)\n', intent_schedule{1,2}, enforced.delay, enforced.loss, enforcement_note);

%% ------------------- Controller selection based on enforced QoS -----------
K = select_controller(K_nom, K_safe, enforced, Ts);

fprintf('Initial controller selected (based on enforced QoS).\n');

%% ------------------- Run simulation loop -------------------------------
for k = 1:N
    tk = t(k);
    
    % --- Check for intent schedule updates ---
    if next_intent_idx < size(intent_schedule,1) && tk >= intent_schedule{next_intent_idx+1,1}
        next_intent_idx = next_intent_idx + 1;
        current_intent = parse_intent_nl(intent_schedule{next_intent_idx,2});
        fprintf('\nIntent change at t=%.2fs: "%s"\n', tk, intent_schedule{next_intent_idx,2});
        % Recompute achievable QoS (could be dynamic; we keep network same but could alter load)
        ach_delay = achievableDelay(network);
        ach_loss  = achievableLoss(network);
        [enforced, enforcement_note] = negotiate_intent(current_intent, ach_delay, ach_loss);
        fprintf(' New enforcement -> delay=%.3fs, loss=%.1e (%s)\n', enforced.delay, enforced.loss, enforcement_note);
        % Update controller selection based on new enforced QoS
        K = select_controller(K_nom, K_safe, enforced, Ts);
        fprintf(' Controller re-selected at t=%.2fs\n', tk);
    end
    
    % --- Plant true output & sensor measurement (position only) ---
    y = Cd*x + Dd*u;             % plant output (position)
    meas = y + sqrt(V)*randn;    % measurement with noise
    
    % Sensor -> sends measurement over network (we simulate packet loss/delay)
    % place measured state (we only send measured output; controller reconstructs state via KF)
    if rand > network.base_loss
        % push measurement into sensor buffer (store scalar)
        sensor_buffer = [meas sensor_buffer(1:end-1)];
    end
    
    % determine arrival of measurement after enforced delay
    delay_steps = min(max_delay_steps, round(enforced.delay / Ts));
    if delay_steps+1 <= size(sensor_buffer,2)
        meas_candidate = sensor_buffer(delay_steps+1);
        if rand < enforced.loss
            meas_valid = false;
        else
            meas_valid = true;
        end
    else
        meas_candidate = meas;
        meas_valid = true;
    end
    
    % --- Kalman filter prediction & update at controller ---
    % Predict
    xhat = Ad*xhat + Bd*u;
    Ppred = []; %#ok<NASGU>
    if meas_valid
        z = meas_candidate;
        % innovation
        innov = z - Cd*xhat;
        xhat = xhat + L_kf * innov;
    else
        % No measurement -> no update (KF holds prediction)
    end
    
    % Controller computes state-feedback based on xhat and reference
    x_ref = [r(k); 0];
    u_new = -K*(xhat - x_ref);
    
    % Send control command through network (simulate duplicates if very high priority)
    redundancy = 1;
    if strcmpi(current_intent.priority,'safety') || strcmpi(current_intent.priority,'high')
        redundancy = 2; % send duplicate packets when high priority
    end
    % place duplicates into actuator buffer
    for d=1:redundancy
        actuator_buffer = [u_new actuator_buffer(1:end-1)];
    end
    
    if delay_steps+1 <= size(actuator_buffer,2)
        cmd_candidate = actuator_buffer(delay_steps+1);
        if rand < enforced.loss
            control_valid = false;
        else
            control_valid = true;
        end
    else
        cmd_candidate = u_new;
        control_valid = true;
    end
    
    if control_valid
        u = cmd_candidate;
    else
        % hold previous u (ZOH)
    end
    
    % Plant update (discrete)
    x = Ad*x + Bd*u;
    
    % logs
    xlog(:,k) = x;
    xhatlog(:,k) = xhat;
    ulog(:,k) = u;
    measlog(k) = meas;
    esterr_log(:,k) = x - xhat;
    
end

%% --------------------- Plots & Metrics --------------------------
figure('Position',[100 80 900 700]);

subplot(4,1,1);
plot(t, r, '--','LineWidth',1.2); hold on;
plot(t, xlog(1,:), 'LineWidth',1.4);
xlabel('Time (s)'); ylabel('Position (rad)');
legend('Ref','Plant Position','Location','best');
title('Reference vs Plant Output (Position)');
grid on;

subplot(4,1,2);
plot(t, xhatlog(1,:), 'LineWidth',1.1); hold on;
plot(t, xlog(1,:),'--','LineWidth',1.0);
xlabel('Time (s)'); ylabel('Position (rad)');
legend('Estimated Position','True Position','Location','best');
title('Estimation: Kalman Filter Performance (position)');
grid on;

subplot(4,1,3);
plot(t, esterr_log(1,:), 'LineWidth',1.1);
xlabel('Time (s)'); ylabel('Estimation Error');
title('Estimation Error (position)');
grid on;

subplot(4,1,4);
plot(t, ulog, 'LineWidth',1.1);
xlabel('Time (s)'); ylabel('Control Input (voltage)');
title('Control Command');
grid on;

% Compute performance metrics before/after intent events
event_times = cell2mat(intent_schedule(:,1))';
fprintf('\n--- Performance summary per phase ---\n');
phase_starts = [0 event_times(2:end)]; % phases start times
phase_ends = [event_times(2:end) Tstop];
for i=1:length(phase_starts)
    idx = find(t >= phase_starts(i) & t < phase_ends(i));
    ess = mean(abs(r(idx) - xlog(1,idx)));
    mse_est = mean((xlog(1,idx) - xhatlog(1,idx)).^2);
    fprintf('Phase %d (t=%.1f-%.1f s): mean abs pos error = %.4f, mean sq est err = %.6f\n', i, phase_starts(i), phase_ends(i), ess, mse_est);
end

fprintf('\nFinal enforced QoS: delay=%.3fs, loss=%.1e (%s)\n', enforced.delay, enforced.loss, enforcement_note);

%% ---------------------- Helper functions ------------------------------

function intent = parse_intent_nl(text)
% Mock LLM parser: extracts desired_delay (s), desired_loss (prob), priority, bandwidth
    intent.desired_delay = 0.015;
    intent.desired_loss  = 1e-4;
    intent.priority = 'normal';
    intent.desired_bandwidth = 1;
    txt = lower(text);
    % extract numeric delay in ms or s
    tokens = regexp(txt, '(\d+\.?\d*)\s*(ms|s)', 'tokens');
    if ~isempty(tokens)
        for i=1:length(tokens)
            val = str2double(tokens{i}{1});
            unit = tokens{i}{2};
            if strcmp(unit,'ms')
                intent.desired_delay = val/1000;
                break;
            elseif strcmp(unit,'s')
                intent.desired_delay = val;
                break;
            end
        end
    end
    if contains(txt,'very high') || contains(txt,'very reliable') || contains(txt,'100%')
        intent.desired_loss = 1e-5;
    elseif contains(txt,'high reliability') || contains(txt,'highly reliable')
        intent.desired_loss = 1e-4;
    elseif contains(txt,'best effort')
        intent.desired_loss = 1e-3;
    end
    if contains(txt,'safety') || contains(txt,'critical') || contains(txt,'emergency')
        intent.priority = 'safety';
    end
end

function [enforced, note] = negotiate_intent(intent, ach_delay, ach_loss)
% Compare desired vs achievable and propose enforcement (with simple redundancy)
    enforced = struct();
    note = 'exact';
    enforced.delay = min(intent.desired_delay, ach_delay); % cannot be less than achievable (we use min to show best effort)
    % If desired is stricter (smaller) than achievable, we mark note as relaxed and propose ach_delay
    if intent.desired_delay < ach_delay
        enforced.delay = ach_delay;
        note = 'relaxed';
    end
    % Loss negotiation: can add redundancy if high priority
    redundancy = 1;
    if strcmpi(intent.priority,'safety')
        redundancy = 2;
    end
    eff_loss = ach_loss^redundancy; % independent duplication approx
    if intent.desired_loss < eff_loss
        % cannot meet desired, enforce best-effort (eff_loss)
        enforced.loss = eff_loss;
        note = 'relaxed';
    else
        enforced.loss = min(intent.desired_loss, eff_loss);
    end
end

function Ksel = select_controller(K_nom, K_safe, enforced, Ts)
% Heuristic controller selection: if enforced delay is large or loss is high -> pick safe
    delay_steps = round(enforced.delay / Ts);
    if enforced.loss > 1e-3 || delay_steps > 6
        Ksel = K_safe;
    else
        % blend nominal and safe depending on severity
        alpha = min(1, delay_steps/6);
        Ksel = (1-alpha)*K_nom + alpha*K_safe;
    end
end
