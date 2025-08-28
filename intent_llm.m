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