clear; clc; close all;

%% I DONT KNOW HOW TO STOP THIS CODE WHEN YOU RUN IT. 
% If you press run, you have to close matlab to stop it.

computeAndStoreResult()



function computeAndStoreResult()
    x = [1,2,3,4,5,6];

    % Initialize a variable to store the result
    result1 = 0;
    
    % Create a timer object that executes the computation every 1 second
    t = timer('ExecutionMode', 'fixedRate', 'Period', 1, 'TimerFcn', @computeResult);
    
    % Start the timer
    start(t);


    function computeResult(~, ~)
        if result1 < length(x)
            % Perform the computation (1 + 1) and store the result
            result1 = result1 + 1;
            disp(x(result1));
        else
            disp('0')
        end
    end

end
