clc;
close all;
clear all;

% Example 1: Servo control
intent = "Rotate servo to 45 degrees, wait 2 seconds, then return to 0 degrees.";
actions = intent_to_actions(intent);

% Example 2: UAV navigation
intent = "Fly drone forward 10 meters, then ascend 5 meters, then hover.";
actions = intent_to_actions(intent);

disp(actions);



function actions = intent_to_actions(intent)
    % === Paste your OpenAI API key here (or use getenv if set) ===
    apiKey = getenv("OPENAI_API_KEY");  % <-- replace with your API key
    
    % API endpoint
    url = "https://api.openai.com/v1/chat/completions";

    % Prepare request data
    data = struct();
    data.model = "gpt-4o-mini";   % use gpt-4o-mini (cheaper & fewer rate limits)
    data.messages = { ...
        struct("role","system","content", ...
               "You are an intent-to-action translator. Translate user intent into a JSON sequence of executable actions."), ...
        struct("role","user","content", intent) ...
    };

    % Encode JSON body
    body = jsonencode(data);

    % HTTP options
    opts = weboptions( ...
        "HeaderFields", ["Authorization", "Bearer " + apiKey; ...
                         "Content-Type","application/json"], ...
        "ContentType","json", ...
        "Timeout", 60);

    % === Retry logic for 429 Too Many Requests ===
    maxRetries = 5;
    waitTime = 2; % start with 2 seconds

    for attempt = 1:maxRetries
        try
            response = webwrite(url, body, opts);
            break; % success → exit loop
        catch ME
            if contains(ME.message,"429") % rate limit error
                fprintf("Rate limit hit (429). Retrying in %d seconds... (Attempt %d/%d)\n", ...
                        waitTime, attempt, maxRetries);
                pause(waitTime);
                waitTime = waitTime * 2; % exponential backoff
            else
                rethrow(ME); % different error → rethrow
            end
        end
        if attempt == maxRetries
            error("Failed after %d retries due to repeated 429 errors.", maxRetries);
        end
    end

    % Extract model output
    rawText = response.choices(1).message.content;
    fprintf("LLM Output:\n%s\n", rawText);

    % Try to parse JSON, else return plain text
    try
        actions = jsondecode(rawText);
    catch
        actions = rawText;
    end
end
