clc;
close all;
clear all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Example usage of intent_to_actions_gemini.m
rawText = generate_with_gemini("Rotate servo 180 degrees, wait 40 seconds and then to 90 by 60 seconds");

function rawText = generate_with_gemini(prompt)
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
        rawText = response.candidates(1).content.parts(1).text;
        disp(rawText);
    else
        disp("No candidates returned.");
    end
end


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Decode JSON
cmds = jsondecode(rawText);

% Possible field names Gemini might use
possibleFields = ["servo", "command", "commands", "servoCommands", "servo_actions", "servo_movements"];

% Initialize empty
actions = [];

% Check which field exists
for f = possibleFields
    if isfield(cmds, f)
        actions = cmds.(f);
        break;
    end
end

% If still empty, maybe nested differently (e.g., cmds.servo.angle)
if isempty(actions)
    warning("No recognized servo command field found.");
else
    % Loop through extracted actions
    for k = 1:length(actions)
        % Sometimes Gemini uses "time", sometimes "duration" or "wait"
        if isfield(actions(k), "time")
            duration = actions(k).time;
        elseif isfield(actions(k), "duration")
            duration = actions(k).duration;
        elseif isfield(actions(k), "wait")
            duration = actions(k).wait;
        else
            duration = NaN; % fallback
        end

        % Angle field is usually consistent
        if isfield(actions(k), "angle")
            angle = actions(k).angle;
        else
            angle = NaN;
        end
        
        fprintf('Action %d: angle = %d°, duration = %d\n', k, angle, duration);
        angles_deg(k) = angle;
        dur_ms(k)     = duration*1000;
    end
    
end

% Count number of commands
if isempty(actions)
    numCmds = 0;
else
    numCmds = length(actions);
end
fprintf('Number of commands = %d\n', numCmds);
dur_ms(dur_ms <= 0) = 100;  % replace 0 duration with small hold

% cumulative waypoint times
waypoint_t = [0, cumsum(dur_ms)/1000];  % seconds
waypoint_angles = [0, angles_deg];      % start from 0 deg

% discrete-time vector and reference trajectory
dt = 0.01;                               % sample period
t_final = waypoint_t(end) + 0.5;         % extra settle time
t = 0:dt:t_final;

% Continous
%ref_deg = interp1(waypoint_t, waypoint_angles, t, 'linear', 'extrap');
% Discrete
ref_deg = [0,waypoint_angles(2)*ones(1,(length(t)-1)/numCmds),waypoint_angles(3)*ones(1,(length(t)-1)/numCmds)];
ref_rad = deg2rad(ref_deg);

%% === Continuous-time second-order servo model ===
J = 1e-3;     % inertia
b = 1e-2;     % damping
K_t = 1e-2;   % torque constant

A_c = [0 1; 0 -b/J];
B_c = [0; K_t/J];
C_c = [1 0];
D_c = 0;

sys_c = ss(A_c,B_c,C_c,D_c);
sys_d = c2d(sys_c, dt);
A_d = sys_d.A; B_d = sys_d.B; C_d = sys_d.C;

%% === Augment with integral action for setpoint tracking ===
A_aug = [A_d, zeros(2,1);
         -C_d, 1];
B_aug = [B_d; 0];
Q = diag([50, 1, 200]);
R = 1e-2;
K_aug = dlqr(A_aug,B_aug,Q,R);

Kx = K_aug(1:2);
Ki = K_aug(3);

%% === Simulation ===
N = length(t);
x = zeros(2,N);
y = zeros(1,N);
u_hist = zeros(1,N);
intErr = 0;

for k = 1:N-1
    r_k = ref_rad(k);
    y(k) = C_d*x(:,k);
    e = r_k - y(k);
    intErr = intErr + e*dt;
    
    u = -Kx*x(:,k) - Ki*intErr;
    u = max(min(u,5),-5);
    u_hist(k) = u;
    
    x(:,k+1) = A_d*x(:,k) + B_d*u;
end
y(N) = C_d*x(:,N);
y_deg = rad2deg(y);

%% === Plots: tracking and control ===
figure;
subplot(2,1,1);
plot(t, ref_deg,'r--','LineWidth',1.5); hold on;
plot(t, y_deg,'b-','LineWidth',1.8);
xlabel('Time (s)'); ylabel('Angle (deg)');
legend('Desired','Tracked'); grid on;
title('Tracking with Discrete LQR + Integral Action');

subplot(2,1,2);
plot(t,u_hist,'k','LineWidth',1.5);
xlabel('Time (s)'); ylabel('Control input');
title('Control Signal'); grid on;

%% === Fixed 4-quadrant animation ===
L = 0.08;  % arm length
fig = figure('Name','Servo: Animation + Angle Plot','Units','normalized','Position',[0.1 0.1 0.6 0.7]);

% --- Top: animation axes
ax1 = subplot(2,1,1); hold(ax1,'on'); axis(ax1,'equal');
margin = 0.02;
xlim(ax1,[-L-margin L+margin]); ylim(ax1,[-L-margin L+margin]);
plot(ax1,[-L-margin L+margin],[0 0],':k','LineWidth',0.8);
plot(ax1,[0 0],[-L-margin L+margin],':k','LineWidth',0.8);
theta_circ = linspace(0,2*pi,400);
plot(ax1, L*cos(theta_circ), L*sin(theta_circ), 'Color',[0.85 0.85 0.85]);
plot(ax1,0,0,'ko','MarkerFaceColor','k','MarkerSize',6);
hActual  = plot(ax1,[0 L],[0 0],'b-','LineWidth',6);
hDesired = plot(ax1,[0 L],[0 0],'r--','LineWidth',2);
hText    = text(ax1, -L+0.01, L-0.01, '', 'FontSize',11);

title(ax1,'Servo Arm (blue = actual, red = desired)');
xlabel(ax1,'X (m)'); ylabel(ax1,'Y (m)'); grid(ax1,'on');

% --- Bottom: angle vs time plot
ax2 = subplot(2,1,2); hold(ax2,'on');
hRef = plot(ax2, t, ref_deg, 'r--','LineWidth',1.4);
hOut = plot(ax2, t, y_deg,  'b-','LineWidth',1.6);
yl = ylim(ax2);
hTime = plot(ax2, [t(1) t(1)], yl,'k--','LineWidth',1.2);
xlabel(ax2,'Time (s)'); ylabel(ax2,'Angle (deg)');
legend(ax2, {'Desired','Actual'}, 'Location','best');
title(ax2,'Angle vs Time'); grid(ax2,'on');
ymin = min([ref_deg(:); y_deg(:)]); ymax = max([ref_deg(:); y_deg(:)]);
pad = max(5, 0.1*(ymax-ymin));
ylim(ax2, [ymin-pad, ymax+pad]);

% --- Animation loop
N = length(t);
playbackSpeed = 1; frameStep = 1;

for k = 1:frameStep:N
    thA = deg2rad(y_deg(k));
    thD = deg2rad(ref_deg(k));
    xA = L*cos(thA); yA = L*sin(thA);
    xD = L*cos(thD); yD = L*sin(thD);
    
    set(hActual,  'XData',[0 xA],'YData',[0 yA]);
    set(hDesired, 'XData',[0 xD],'YData',[0 yD]);
    set(hText, 'String', sprintf('t=%.2fs | Actual=%.1f° | Desired=%.1f°', t(k), y_deg(k), ref_deg(k)));
    
    set(hTime, 'XData',[t(k) t(k)], 'YData', ylim(ax2));
    
    drawnow limitrate;
    if k + frameStep <= N
        pause( (t(min(k+frameStep,N)) - t(k))/playbackSpeed );
    end
end

%% === Performance metric ===
err = ref_deg - y_deg;
rmse = sqrt(mean(err.^2));
fprintf('Tracking RMSE = %.3f deg\n', rmse);
