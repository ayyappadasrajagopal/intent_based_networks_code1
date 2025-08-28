clc;
close all;
clear all;

% Example usage of intent_to_actions_gemini.m
generate_with_gemini("Rotate servo 90 degrees, wait 2 seconds, then return to 0.");

function generate_with_gemini(prompt)
    % Set your Gemini API Key (get it from AI Studio: https://aistudio.google.com/app/apikey)
    apiKey = getenv("GEMINI_API_KEY");
    if isempty(apiKey)
        error("GEMINI_API_KEY environment variable not set. Run: setx GEMINI_API_KEY ""your_key_here"" ");
    end

    % Gemini endpoint
    url = "https://generativelanguage.googleapis.com/v1beta/models/gemini-1.5-flash:generateContent?key=" + apiKey;

    % Build request JSON
    data = struct( ...
        "contents", {struct( ...
            "role","user", ...
            "parts", {struct("text", prompt)} ...
        )}, ...
        "generationConfig", struct( ...
            "response_mime_type","application/json" ...
        ) ...
    );

    % Convert to JSON string
    body = jsonencode(data);

    % Web options
    opts = weboptions( ...
        "MediaType","application/json", ...
        "Timeout",60 ...
    );

    % Make the request
    response = webwrite(url, body, opts);

    % Extract candidates
    if isfield(response,"candidates") && ~isempty(response.candidates)
        disp("---- Gemini Response ----");
        disp(response.candidates(1).content.parts(1).text);
    else
        disp("No candidates returned.");
    end
end


rawText = '{"commands": [{"servo": {"angle": 90, "time": 2000}}, {"servo": {"angle": 0, "time": 0}}]}';
cmds = jsondecode(rawText);

% Access first command
disp(cmds.commands(1).servo.angle);  % → 90
disp(cmds.commands(1).servo.time);   % → 2000


%% === Parse JSON into MATLAB struct ===
cmdStruct = jsondecode(rawText);

% Validate structure
if ~isfield(cmdStruct, 'commands')
    error('JSON does not contain "commands" field.');
end

% Build reference trajectory: piecewise constant reference with hold times (ms)
% We interpret each command's "angle" as degrees and "time" as dwell in ms.
refAngles_deg = [];
refTimes_s = [];

t_cursor = 0;
hold_default = 0.5; % seconds if time==0 (a small default hold)
for i = 1:numel(cmdStruct.commands)
    c = cmdStruct.commands(i).servo;
    if ~isfield(c, 'angle') || ~isfield(c,'time')
        error('Each servo command must have angle and time fields');
    end
    angle = double(c.angle);          % degrees
    dwell_ms = double(c.time);        % milliseconds
    if dwell_ms <= 0
        dwell_s = hold_default;
    else
        dwell_s = dwell_ms/1000;
    end

    % Add a small smooth ramp (0.05 s) before hold to avoid infinite derivative
    ramp = 0.05;
    % Append "go to angle" (instant step in reference; controller will track)
    refAngles_deg = [refAngles_deg, angle]; %#ok<*AGROW>
    refTimes_s  = [refTimes_s, t_cursor + ramp];  % time at end of ramp
    t_cursor = t_cursor + ramp;

    % Append hold interval end
    refAngles_deg = [refAngles_deg, angle];
    refTimes_s  = [refTimes_s, t_cursor + dwell_s];
    t_cursor = t_cursor + dwell_s;
end

% Create a dense time vector for simulation
t_sim = linspace(0, t_cursor, max(400,ceil(200*t_cursor))); % at least 400 samples

% Build piecewise-constant reference over t_sim (use interp1 with 'previous')
ref_deg = interp1(refTimes_s, refAngles_deg, t_sim, 'previous', 'extrap');
% For the very start, ensure it begins at initial angle (use first)
if t_sim(1) == 0
    ref_deg(1) = refAngles_deg(1);
end
ref_rad = deg2rad(ref_deg);  % convert to radians

%% === Servo / motor state-space model ===
% Simple rotational second-order model:
%   J * theta_dd + b * theta_d = K_t * u
% state x = [theta; theta_dot]
% xdot = [theta_dot; (-b/J)*theta_dot + (K_t/J) * u]

J = 2.5e-4;     % moment of inertia (kg*m^2) -- adjust for your virtual servo
b = 5e-6;       % viscous damping
K_t = 1e-3;     % control gain (torque per command unit) - you can scale u to torque

% Controller gains (PD)
Kp = 60;   % proportional
Kd = 2;    % derivative

% Saturation for input (command), expressed in same units as u (we will limit u)
u_max = 2.0;    % max control command
u_min = -2.0;

% Interpolate reference for use inside ODE
ref_fun = @(t) interp1(t_sim, ref_rad, t, 'previous', ref_rad(1));

% For derivative of ref (zero for piecewise-constant except at transitions)
% We'll compute derivative numerically on the fly using small dt:
ref_dot_fun = @(t) 0;  % reference is piecewise constant -> derivative zero (PD still works)

% ODE for states with feedback control
function dx = servo_ode(t, x)
    theta = x(1);
    theta_dot = x(2);

    r = ref_fun(t);
    r_dot = ref_dot_fun(t);

    % PD control law (on angle error)
    e = r - theta;
    edot = r_dot - theta_dot;

    % control 'u' (command) before gain K_t
    u = Kp*e + Kd*edot;

    % saturate
    u = min(max(u, u_min), u_max);

    % dynamics
    theta_dd = (-b/J)*theta_dot + (K_t/J) * u;

    dx = [theta_dot; theta_dd];
end

% Initial state: assume starting at first reference
x0 = [ref_rad(1); 0];

% Solve ODE with ode45 across time vector (use event-free integration)
opts = odeset('RelTol',1e-6,'AbsTol',1e-8);
[tt, xx] = ode45(@servo_ode, t_sim, x0, opts);

theta_sim = xx(:,1);         % rad
theta_dot_sim = xx(:,2);     % rad/s

% Recompute control input history (so we can plot torque/u)
u_hist = zeros(size(tt));
for k = 1:length(tt)
    r = interp1(t_sim, ref_rad, tt(k), 'previous', ref_rad(1));
    r_dot = 0;
    e = r - theta_sim(k);
    edot = r_dot - theta_dot_sim(k);
    u = Kp*e + Kd*edot;
    u = min(max(u, u_min), u_max);
    u_hist(k) = u;
end

%% === Animate servo motion and plot results ===
% Prepare figure
figure('Name','Servo Simulation','Units','normalized','Position',[0.05 0.05 0.9 0.8]);

% Subplot A: animation axes
ax1 = subplot(2,2,[1 3]);
axis equal
axis([-0.12 0.12 -0.12 0.12]);
hold on;
grid on;
title('Servo Arm Animation');
xlabel('X (m)'); ylabel('Y (m)');
% Servo arm length (m)
L = 0.08;
% Base origin
base = [0,0];

% Plot static base
plot(0,0,'ko','MarkerSize',10,'MarkerFaceColor','k');

% Create line object for arm
armLine = plot([0 L],[0 0],'-','LineWidth',4);
pivot = plot(0,0,'ko','MarkerFaceColor','k','MarkerSize',6);

% Subplot B: angle plot
ax2 = subplot(2,2,2);
hold on; grid on;
hRef = plot(t_sim, rad2deg(ref_rad),'--','LineWidth',1.2);
hSim = plot(tt, rad2deg(theta_sim),'-','LineWidth',2);
xlabel('Time (s)'); ylabel('Angle (deg)');
title('Reference vs Simulated Angle');
legend([hRef hSim],{'Reference','Simulated'},'Location','best');

% Subplot C: control input plot
ax3 = subplot(2,2,4);
hold on; grid on;
hU = plot(tt, u_hist, '-','LineWidth',1.5);
xlabel('Time (s)'); ylabel('Control command (u)');
title('Control Input');

% Animation loop: step through simulation and animate
frames = length(tt);
for k = 1:frames
    th = theta_sim(k);
    x = L*cos(th);
    y = L*sin(th);
    set(armLine, 'XData', [0 x], 'YData', [0 y]);
    set(pivot, 'XData', 0, 'YData', 0);
    % update time marker on plots
    % show vertical line on angle plot
    if exist('vh','var') && isvalid(vh)
        delete(vh);
    end
    vh = line(ax2, [tt(k) tt(k)], ax2.YLim, 'Color', [0.5 0.5 0.5], 'LineStyle', '--');
    % same for u plot
    if exist('vh2','var') && isvalid(vh2)
        delete(vh2);
    end
    vh2 = line(ax3, [tt(k) tt(k)], ax3.YLim, 'Color', [0.5 0.5 0.5], 'LineStyle', '--');

    drawnow;
    % Slow animation to real time approximately
    if k < frames
        pause((tt(k+1)-tt(k))/2); % speed factor: use /1 for real-time, /2 for faster
    end
end

% Final printout
fprintf('Simulation complete. Total simulated time: %.3f s\n', tt(end));