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



function actions = intent_to_actions_gemini(intent)
    apiKey = "YOUR_GEMINI_API_KEY";  % paste your Gemini key here
    url = "https://generativelanguage.googleapis.com/v1beta/openai/chat/completions";

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
