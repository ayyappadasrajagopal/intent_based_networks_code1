% ibn_ss_servo_lqr_kf.m
% Discrete state-space DC servo motor + LQR + Kalman filter + mock-LLM intent parsing
% Demonstrates how natural-language intents change LQR weights and (optionally)
% network QoS (measurement delay/loss), and how that affects control & estimation.
%
clear; close all; clc;

%% -------------------- Plant (continuous) - DC servo state-space ----------
% Physical parameters (example)
J = 0.01;    % inertia (kg.m^2)
b = 0.1;     % damping (Nms)
Kt = 0.01;   % motor constant (Nm/V)
% States: x = [theta; omega], output y = theta (position)
Acont = [0 1; 0 -b/J];
Bcont = [0; Kt/J];
Ccont = [1 0];
Dcont = 0;

% Discretize
Ts = 0.01; % sampling time, 10 ms
sysc = ss(Acont, Bcont, Ccont, Dcont);
sysd = c2d(sysc, Ts);
Ad = sysd.A; Bd = sysd.B; Cd = sysd.C; Dd = sysd.D;

nx = size(Ad,1); nu = size(Bd,2); ny = size(Cd,1);

%% -------------------- Kalman filter design (steady-state) ----------------
% Noise covariances (tunable)
Qw = 1e-5 * eye(nx);  % process noise covariance
Rv = 5e-4;             % measurement noise variance (position sensor)

% Compute steady-state Kalman gain by solving Riccati (discrete)
% Use dare to compute estimator error covariance: P = A P A' - A P C' (C P C' + R)^-1 C P A' + Q
% We'll iterate to steady state
P = eye(nx)*1e-3;
for iter = 1:200
    S = Cd*P*Cd' + Rv;
    Kk = P*Cd'/S;
    P = Ad*(P - Kk*Cd*P)*Ad' + Qw;
end
L_kf = Kk; % estimator gain (state correction: xhat = Ad*xhat + Bd*u + L*(y - C*xhat))

%% -------------------- LQR nominal & safe design templates ----------------
% Define three sets to be used by LLM-to-weights mapping
Q_template_fast  = diag([500, 20]); R_template_fast  = 0.001;   % aggressive -> fast response
Q_template_bal   = diag([150, 5]);  R_template_bal   = 0.01;    % balanced
Q_template_energy= diag([30, 1]);   R_template_energy= 0.5;     % energy saving -> smooth control

% Precompute LQR gains for each template (discrete dlqr)
try
    K_fast  = dlqr(Ad, Bd, Q_template_fast, R_template_fast);
    K_bal   = dlqr(Ad, Bd, Q_template_bal,  R_template_bal);
    K_energy= dlqr(Ad, Bd, Q_template_energy, R_template_energy);
catch
    warning('dlqr not available — falling back to heuristic gains.');
    K_fast  = [45 8];
    K_bal   = [18 3.5];
    K_energy= [6 1.2];
end

%% -------------------- Simulation settings --------------------------------
Tfinal = 12;               % seconds
N = round(Tfinal / Ts);
t = (0:N-1)*Ts;

% Reference trajectory: step then change
r = zeros(1,N);
r(t >= 0.5) = 0.6;   % step to 0.6 rad
r(t >= 7.0) = -0.4;  % later change to -0.4 rad

% Initial conditions
x = [0; 0];
xhat = [0; 0];
u = 0;

% Buffers for delayed measurements (max delay cap)
max_delay_steps = 80;
meas_buffer = nan(1, max_delay_steps+5);
cmd_buffer = nan(1, max_delay_steps+5);

% Logging
xlog = zeros(nx, N);
xhatlog = zeros(nx, N);
ulog = zeros(nu, N);
measlog = nan(1,N);

% RNG for reproducibility
rng(0);

%% -------------------- Intent schedule (natural-language) -------------------
% Each row: [time_when_active, natural-language intent string]
intent_schedule = {
    0.0, "Fast response: fast, prioritize speed over energy; target delay < 15 ms; high reliability";
    4.0, "Balanced: prefer smoothness and some energy saving; allow delay up to 40 ms; high reliability";
    8.5, "Energy saving: minimize control energy, tolerate slower response; best-effort reliability"
};

next_intent_idx = 1;
current_text_intent = intent_schedule{next_intent_idx,2};
current_intent = nl_to_intent(current_text_intent); % mock LLM parser
[enforced_QR, enforced_qos] = enforce_intent(current_intent); % map to controller + QoS
K_current = enforced_QR.K;
fprintf('t=0.00s Active intent: "%s"\n  -> selected controller: %s, enforced measurement delay=%.1f ms, loss=%.1e\n',...
    current_text_intent, enforced_QR.name, enforced_qos.delay*1000, enforced_qos.loss);

%% -------------------- Main simulation loop -------------------------------
for k = 1:N
    tk = t(k);
    % check for intent change
    if next_intent_idx < size(intent_schedule,1) && tk >= intent_schedule{next_intent_idx+1,1}
        next_intent_idx = next_intent_idx + 1;
        current_text_intent = intent_schedule{next_intent_idx,2};
        current_intent = nl_to_intent(current_text_intent);
        [enforced_QR, enforced_qos] = enforce_intent(current_intent);
        K_current = enforced_QR.K;
        fprintf('\nIntent change at t=%.2f s: "%s"\n  -> selected controller: %s, enforced delay=%.1f ms, loss=%.1e\n',...
            tk, current_text_intent, enforced_QR.name, enforced_qos.delay*1000, enforced_qos.loss);
    end

    % --- Plant measurement (position) with sensor noise ---
    y = Cd*x + Dd*u;
    meas = y + sqrt(Rv)*randn;  % noisy measurement

    % simulate sending measurement into network (may be dropped due to base network loss)
    % We assume sensor can always place measurement onto network buffer; enforcement controls delay/loss
    meas_buffer = [meas meas_buffer(1:end-1)];

    % Determine arrival after enforced delay
    delay_steps = min(max_delay_steps, round(enforced_qos.delay / Ts));
    if delay_steps+1 <= length(meas_buffer)
        meas_candidate = meas_buffer(delay_steps+1);
        % simulate enforced packet loss
        if rand < enforced_qos.loss
            meas_valid = false;
        else
            meas_valid = true;
        end
    else
        meas_candidate = meas;
        meas_valid = true;
    end

    % --- Kalman filter predict & update at controller ---
    % predict
    xhat = Ad*xhat + Bd*u;
    if meas_valid
        z = meas_candidate;
        innov = z - Cd*xhat;
        xhat = xhat + L_kf * innov;
    else
        % no update; estimator runs open-loop for this step
    end

    % --- LQR control law (state-feedback with reference) ---
    x_ref = [r(k); 0];
    u_new = -K_current * (xhat - x_ref);

    % simulate actuator network (we assume actuator path has same delay/loss as measurement path)
    % place into cmd buffer (no duplication here but could be added for critical events)
    cmd_buffer = [u_new cmd_buffer(1:end-1)];
    if delay_steps+1 <= length(cmd_buffer)
        cmd_candidate = cmd_buffer(delay_steps+1);
        if rand < enforced_qos.loss
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

    % --- Plant update (discrete) ---
    x = Ad*x + Bd*u;

    % --- Logging ---
    xlog(:,k) = x;
    xhatlog(:,k) = xhat;
    ulog(:,k) = u;
    measlog(k) = meas_valid * meas_candidate + (~meas_valid)*NaN;
end

%% -------------------- Plots & performance metrics -----------------------
figure('Position',[120 80 1000 800]);

subplot(4,1,1);
plot(t, r,'--','LineWidth',1.2); hold on;
plot(t, xlog(1,:),'LineWidth',1.3);
ylabel('Position (rad)'); legend('Ref','Plant'); title('Reference vs Plant Position');
grid on;

subplot(4,1,2);
plot(t, xhatlog(1,:), 'LineWidth',1.1); hold on;
plot(t, xlog(1,:), '--', 'LineWidth',1.0);
ylabel('Position (rad)'); legend('Estimate','True'); title('Kalman Estimate vs True Position');
grid on;

subplot(4,1,3);
plot(t, xlog(1,:) - xhatlog(1,:), 'LineWidth',1.0);
ylabel('Estimation error (pos)'); title('Estimation Error (position)'); grid on;

subplot(4,1,4);
plot(t, ulog, 'LineWidth',1.1);
xlabel('Time (s)'); ylabel('Control (V)'); title('Control Input'); grid on;

% Phase-wise metrics
phase_times = cell2mat(intent_schedule(:,1))';
phase_starts = [0 phase_times(2:end)];
phase_ends = [phase_times(2:end) Tfinal];
fprintf('\n--- Phase-wise performance summary ---\n');
for i = 1:length(phase_starts)
    idx = find(t >= phase_starts(i) & t < phase_ends(i));
    mean_abs_err = mean(abs(r(idx) - xlog(1,idx)));
    mse_est = mean((xlog(1,idx) - xhatlog(1,idx)).^2);
    mean_u = mean(abs(ulog(idx)));
    fprintf('Phase %d (t=%.1f-%.1f s): mean abs pos err = %.4f, mse est err = %.6e, mean|u| = %.4f\n',...
        i, phase_starts(i), phase_ends(i), mean_abs_err, mse_est, mean_u);
end

%% -------------------- Helper functions ----------------------------------

function intent = nl_to_intent(nl_text)
% Mock LLM parser: convert natural language intent -> structured fields
% Fields: mode ('fast'|'balanced'|'energy'), desired_delay (s), desired_loss (prob), priority
    t = lower(nl_text);
    intent = struct();
    intent.mode = 'balanced';
    intent.desired_delay = 0.02; % default 20 ms
    intent.desired_loss  = 1e-4; % default
    intent.priority = 'normal';
    if contains(t,'fast')
        intent.mode = 'fast';
        intent.desired_delay = 0.012; % 12 ms
        intent.desired_loss = 1e-5;
    elseif contains(t,'balanced') || contains(t,'balance')
        intent.mode = 'balanced';
        intent.desired_delay = 0.03; % 30 ms
        intent.desired_loss = 5e-5;
    elseif contains(t,'energy')
        intent.mode = 'energy';
        intent.desired_delay = 0.05; % 50 ms (slower)
        intent.desired_loss = 1e-3;  % best-effort
    end
    if contains(t,'high reliability') || contains(t,'very high reliability') || contains(t,'high reliability') || contains(t,'safety')
        intent.desired_loss = min(intent.desired_loss, 1e-5);
        intent.priority = 'high';
    end
    % try to parse explicit milliseconds if provided
    tok = regexp(t, '(\d+\.?\d*)\s*(ms|s)', 'tokens');
    if ~isempty(tok)
        for i=1:length(tok)
            val = str2double(tok{i}{1});
            unit = tok{i}{2};
            if strcmp(unit,'ms')
                intent.desired_delay = val/1000;
                break;
            elseif strcmp(unit,'s')
                intent.desired_delay = val;
                break;
            end
        end
    end
end

function [controller_struct, qos] = enforce_intent(intent)
% Map intent -> select controller gain (from precomputed templates) and enforce QoS
% Here we pick among precomputed K_fast, K_bal, K_energy stored in caller workspace
    % Access templates from base workspace (precomputed)
    K_fast_local = evalin('base','K_fast'); 
    K_bal_local  = evalin('base','K_bal');
    K_energy_local = evalin('base','K_energy');
    % Choose controller based on intent.mode
    switch intent.mode
        case 'fast'
            controller_struct.K = K_fast_local;
            controller_struct.name = 'fast';
        case 'energy'
            controller_struct.K = K_energy_local;
            controller_struct.name = 'energy';
        otherwise
            controller_struct.K = K_bal_local;
            controller_struct.name = 'balanced';
    end
    % QoS enforcement (we simulate that desired may be relaxed by phys limits)
    % For simplicity define achievable network capability (could be dynamic)
    ach_delay = 0.015; % 15 ms achievable best-case
    ach_loss  = 5e-5;
    qos.delay = max(intent.desired_delay, ach_delay); % cannot be faster than achievable -> relaxed upward
    % For loss, we try to meet desired; if desired < ach_loss, we set to ach_loss (cannot be lower)
    if intent.desired_loss < ach_loss
        % If high priority, attempt duplication to improve effective reliability (approx)
        if strcmp(intent.priority,'high')
            qos.loss = ach_loss^2; % approximate effect of one duplicate path (independent)
        else
            qos.loss = ach_loss;
        end
    else
        qos.loss = intent.desired_loss;
    end
end
