
% Example 1: Servo control
intent = "Rotate servo to 45 degrees, wait 2 seconds, then return to 0 degrees.";
actions = intent_to_actions(intent);

% Example 2: UAV navigation
intent = "Fly drone forward 10 meters, then ascend 5 meters, then hover.";
actions = intent_to_actions(intent);

disp(actions);


%% Intent → Actions using LLM (OpenAI API)
% Make sure to set your API key first in terminal/PowerShell:
%   setx OPENAI_API_KEY "your_api_key_here"   (Windows)
%   export OPENAI_API_KEY="your_api_key_here" (Linux/Mac)

function actions = intent_to_actions(intent)
    % Read API key
    apiKey = getenv("OPENAI_API_KEY");
    if isempty(apiKey)
        error("No API key found. Please set OPENAI_API_KEY environment variable.");
    end

    % API endpoint
    url = "https://api.openai.com/v1/chat/completions";

    % Define system & user prompts
    data = struct( ...
        "model", "gpt-4o-mini", ... % Or "gpt-4o", "gpt-5" when available
        "messages", { ...
            struct("role","system","content", ...
                   "You are an intent-to-action translator. " + ...
                   "Translate user intent into a JSON sequence of executable actions."), ...
            struct("role","user","content", intent) ...
        } ...
    );

    % Setup options
    opts = weboptions( ...
        "HeaderFields", ["Authorization", "Bearer " + apiKey; ...
                         "Content-Type","application/json"], ...
        "Timeout", 60);

    % Call API
    response = webwrite(url, data, opts);

    % Extract response
    rawText = response.choices(1).message.content;
    fprintf("LLM Output:\n%s\n", rawText);

    % Try to decode JSON if present
    try
        actions = jsondecode(rawText);
    catch
        actions = rawText; % fallback to plain text
    end
end
