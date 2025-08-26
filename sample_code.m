% ibn_lqr_llm_demo.m
% Discrete-state-space NCS demo with:
%  - continuous plant -> discrete LTI model
%  - LLM (mock) that parses natural-language intent -> structured intent
%  - feasibility check vs network capabilities
%  - LQR controller + safe fallback LQR
%  - network delay & loss emulation (sensor->controller & controller->actuator)
%
% Author: ChatGPT (for Ayyappadas) - 2025
clear; close all; clc;

%% ---------------- Parameters / Plant ----------------
% Continuous plant (example second order): G(s) = 1 / (0.5 s^2 + 1 s + 1)
num = 1;
den = [0.5 1 1];
G = tf(num, den);

% Convert to continuous state-space
sysc = ss(G);
[Acont, Bcont, Ccont, Dcont] = ssdata(sysc);

% Discretization / control sampling time
Ts = 0.01;           % controller/communication sampling time (s)
sysd = c2d(sysc, Ts);
Ad = sysd.A; Bd = sysd.B; Cd = sysd.C; Dd = sysd.D;

nx = size(Ad,1);
nu = size(Bd,2);

%% ------------- LQR design (nominal and safe fallback) -------------
% Nominal LQR (aggressive) weightings
Q_nom = diag([400, 10]);   % penalize position error strongly
R_nom = 0.01;

% Safe fallback LQR (conservative)
Q_safe = diag([100, 1]);
R_safe = 0.1;

% Compute gains (requires Control System Toolbox)
try
    K_nom = lqr(Ad, Bd, Q_nom, R_nom);
    K_safe = lqr(Ad, Bd, Q_safe, R_safe);
catch
    % If lqr unavailable, fall back to a heuristic state-feedback (small gains)
    warning('lqr() not available: using heuristic gains.');
    K_nom = [20 4];
    K_safe = [6 1];
end

%% ---------------- Network capability (measured / predicted) ------------
network.base_delay = 0.010;    % base one-way delay (s)
network.base_loss  = 1e-4;     % base packet loss probability
network.load = 0.35;           % 0..1

% Simple model: load -> achievable QoS
achievableDelay = @(net) net.base_delay + 0.03*net.load;   % s
achievableLoss  = @(net) net.base_loss + 5e-4*net.load;    % prob

%% ------------------ User (natural-language) intent -------------------
% Example natural language intent (user)
user_text = "Keep actuator feedback delay below 20 ms, ensure very high reliability for sensors, prioritize safety";

% Parse intent (mock LLM parser)
intent = parse_intent_nl(user_text);
fprintf('User (NL) -> Parsed Intent: delay <= %.1f ms, loss <= %.1e, priority=%s\n',...
    intent.desired_delay*1e3, intent.desired_loss, intent.priority);

%% ------------------ Feasibility Check & Negotiation -------------------
ach_delay = achievableDelay(network);
ach_loss = achievableLoss(network);

fprintf('Network achievable (current): delay = %.3f s, loss = %.1e\n', ach_delay, ach_loss);

feasible.delay = (intent.desired_delay >= ach_delay);
feasible.loss  = (intent.desired_loss >= ach_loss);
feasible.all = feasible.delay && feasible.loss;

if feasible.all
    fprintf('Intent feasible: enforcing requested QoS exactly.\n');
    enforced.delay = intent.desired_delay;
    enforced.loss  = intent.desired_loss;
    enforcement_note = 'exact';
else
    fprintf('Intent NOT fully feasible: negotiating / proposing best-effort.\n');
    % Try redundancy if priority high
    redundancy_factor = 1;
    if strcmpi(intent.priority, 'high') || strcmpi(intent.priority,'safety')
        redundancy_factor = 2;
        fprintf(' Applying redundancy factor = %d for critical traffic.\n', redundancy_factor);
    end
    % Effective loss if duplicates sent (independent path approx): e.g., p_eff = p^k
    eff_loss = (ach_loss)^redundancy_factor;
    enforced.delay = ach_delay; % cannot beat physics
    enforced.loss = min(eff_loss, ach_loss);
    enforcement_note = 'relaxed';
    fprintf(' Proposed enforcement: delay = %.3f s, effective loss = %.1e\n', enforced.delay, enforced.loss);
end

%% ------------- Controller selection & adaptation -------------------
% Heuristic: if enforced delay is larger than Ts * threshold_steps, reduce aggressiveness
delay_steps = round(enforced.delay / Ts);
if delay_steps <= 1
    K = K_nom;
    mode = 'nominal';
else
    % scale the nominal K toward safe K based on delay severity
    alpha = min(1, delay_steps/5); % 0..1, larger -> more towards safe
    K = (1-alpha)*K_nom + alpha*K_safe;
    mode = 'delay-adapted';
end

% If loss is too high, force safe fallback
if enforced.loss > 1e-3
    K = K_safe;
    mode = 'fallback';
end

fprintf('Using controller mode: %s\n', mode);
disp('K matrix:'); disp(K);

%% ------------------- Simulation (closed-loop discrete) ------------------
Tstop = 8;                          % seconds
N = round(Tstop/Ts);
t = (0:N-1)*Ts;

% Reference (step)
r = zeros(1,N);
r(t >= 0.5) = 1;

% Network delay/loss model (apply same enforced QoS)
one_way_delay = enforced.delay;
delay_steps = round(one_way_delay / Ts);   % integer number of samples
loss_prob = enforced.loss;

% Buffers (for symmetric sensor/controller & controller/actuator delays)
sensor_buffer = zeros(nx, delay_steps+3);      % store state vectors
actuator_buffer = zeros(nu, delay_steps+3);

% Initialize states
x = zeros(nx,1);
x_hat = zeros(nx,1);    % controller will use last received state sample (no observer)
u = 0;

% logs
xlog = zeros(nx, N);
ulog = zeros(nu, N);
rlog = r;

rng(1);

for k = 1:N
    % time
    tk = t(k);
    % --- plant output (we assume full-state sensors; sensor sends x)
    y = Cd*x + Dd*u;   %#ok<NASGU>  % plant output (not used directly; we use state)
    
    % Sensor sends state (with some local reliability) -> pushed to buffer
    if rand > network.base_loss % we assume sensor node can send into network
        sensor_buffer = [x sensor_buffer(:,1:end-1)];
    end
    
    % After delay_steps, sensor packet arrives at controller (if not lost)
    if delay_steps+1 <= size(sensor_buffer,2)
        x_candidate = sensor_buffer(:, delay_steps+1);
        if rand < loss_prob
            sensor_valid = false;
        else
            sensor_valid = true;
        end
    else
        x_candidate = x;
        sensor_valid = true;
    end
    
    if sensor_valid
        x_hat = x_candidate;  % controller uses received full state
    else
        % measurement lost: controller holds previous x_hat (zero-order hold)
        % optionally we could predict x_hat using model, but keep it simple here
    end
    
    % Controller computes control u = -K * (x_hat - x_ref_state)
    % For regulation to reference r we need state reference; we convert r to state (only pos)
    x_ref = [r(k); 0];  % assume second state is velocity; reference for velocity zero
    u_new = -K * (x_hat - x_ref);
    
    % Put control(s) into actuator buffer; apply redundancy by repeating entries
    actuator_buffer = [repmat(u_new,1,1) actuator_buffer(:,1:end-1)]; %#ok<AGROW>
    
    % After delay_steps, actuator receives command (if at least one duplicate arrives)
    if delay_steps+1 <= size(actuator_buffer,2)
        cmd_candidate = actuator_buffer(:, delay_steps+1);
        if rand < loss_prob
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
    ulog(:,k) = u;
end

%% ------------------ Plots & summary -------------------------
figure('Position',[100 100 800 520]);
subplot(2,1,1);
plot(t, r, '--','LineWidth',1.2); hold on;
plot(t, xlog(1,:), 'LineWidth',1.4);
xlabel('Time (s)'); ylabel('Position');
title(sprintf('Closed-loop (LQR) response — controller mode: %s — enforced delay=%.3fs, loss=%.1e',...
    mode, enforced.delay, enforced.loss));
legend('Ref','x_1 (position)','Location','best'); grid on;

subplot(2,1,2);
plot(t, ulog, 'LineWidth',1.2);
xlabel('Time (s)'); ylabel('Control u');
title('Control command (actuator log)'); grid on;

% Performance metric
idx_last = find(t >= (Tstop - 1));
ss_err = mean(abs(r(idx_last) - xlog(1,idx_last)));
fprintf('\nSummary:\nRequested delay=%.3fs, requested loss=%.1e\n', intent.desired_delay, intent.desired_loss);
fprintf('Enforced delay=%.3fs, enforced loss=%.1e (%s)\n', enforced.delay, enforced.loss, enforcement_note);
fprintf('Mean steady-state abs. error (last 1s): %.4f\n', ss_err);

%% ------------------ Mock LLM parser (replaceable) ---------------------
function intent = parse_intent_nl(text)
% Very small rule-based NL parser that extracts:
%  - desired_delay (s)
%  - desired_loss (prob)
%  - priority (low/normal/high/safety)
% You can replace this function by calling a real LLM API (OpenAI, local LLaMA, etc.)
%
% Example replacement (pseudo-code):
%   response = llm_api_call(prompt_with_user_text_and_schema);
%   intent = jsondecode(response);

    % defaults
    intent.desired_delay = 0.02;   % 20 ms default
    intent.desired_loss = 1e-4;    % default
    intent.priority = 'normal';
    intent.desired_bandwidth = 1;  % Mbps (informational)
    
    txt = lower(text);
    % extract delay (ms)
    tok = regexp(txt, '(\d+\.?\d*)\s*(ms|s)', 'tokens');
    if ~isempty(tok)
        % find first numeric token with unit
        for i=1:numel(tok)
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
    
    % keywords for reliability / loss
    if contains(txt, 'very high') || contains(txt, 'very reliable') || contains(txt,'very high reliability') || contains(txt,'100%')
        intent.desired_loss = 1e-5;
    elseif contains(txt,'high reliability') || contains(txt,'highly reliable') || contains(txt,'high reliability')
        intent.desired_loss = 1e-4;
    elseif contains(txt,'best effort')
        intent.desired_loss = 1e-3;
    end
    
    % priority
    if contains(txt,'safety') || contains(txt,'critical') || contains(txt,'emergency')
        intent.priority = 'safety';
    elseif contains(txt,'high priority') || contains(txt,'highly')
        intent.priority = 'high';
    elseif contains(txt,'low priority')
        intent.priority = 'low';
    end
    
    % bandwidth (simple)
    bw_tok = regexp(txt, '(\d+\.?\d*)\s*(mbps|mb/s|mb)', 'tokens');
    if ~isempty(bw_tok)
        intent.desired_bandwidth = str2double(bw_tok{1}{1});
    end
end
