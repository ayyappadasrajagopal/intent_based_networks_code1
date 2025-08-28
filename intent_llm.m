clc;
close all;
clear all;

% Example usage of intent_to_actions_gemini.m

% Define the user intent (in natural language)
intent = "Rotate servo to 90 degrees, wait 2 seconds, then return to 0 degrees.";

% Call your function (make sure your Gemini API key is set in the function)
actions = intent_to_actions_gemini(intent);

% Display the returned structure or text
disp("---- Actions from Gemini ----");
disp(actions);

% If it's a struct with steps, loop through them
if isstruct(actions) && isfield(actions,"steps")
    for i = 1:numel(actions.steps)
        step = actions.steps(i);
        fprintf("Step %d: %s %s %s\n", i, ...
            string(step.action), ...
            string(step.value), ...
            string(step.unit));
    end
end

function actions = intent_to_actions_gemini(intent)
    apiKey = getenv("GEMINI_API_KEY");  % paste your Gemini key here
    url = "https://generativelanguage.googleapis.com/v1beta/models/gemini-1.5-flash:generateContent?key=apiKey";

    data = struct( ...
        "model", "gemini-2.5-flash", ...
        "messages", { ...
            struct("role","system","content","Translate intent into JSON actions."), ...
            struct("role","user","content", intent) ...
        } ...
    );

    body = jsonencode(data);
    opts = weboptions( ...
        "HeaderFields", ["Authorization", "Bearer " + apiKey; "Content-Type","application/json"], ...
        "ContentType","json", ...
        "Timeout", 60);

    maxRetries = 5; waitTime = 2;
    for attempt = 1:maxRetries
        try
            response = webwrite(url, body, opts);
            break;
        catch ME
            if contains(ME.message,"429")
                fprintf("429 Too Many Requests—retrying in %d sec (try %d/%d)\n", waitTime, attempt, maxRetries);
                pause(waitTime); waitTime = waitTime*2;
            else
                rethrow(ME);
            end
        end
        if attempt == maxRetries
            error("Exceeded retry limit due to repeated 429 errors.");
        end
    end

    rawText = response.choices(1).message.content;
    fprintf("Gemini Output:\n%s\n", rawText);
    try
        actions = jsondecode(rawText);
    catch
        actions = rawText;
    end
end
