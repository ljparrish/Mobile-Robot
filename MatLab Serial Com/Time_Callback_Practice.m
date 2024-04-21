clear; clc; close all;

%% I DONT KNOW HOW TO STOP THIS CODE WHEN YOU RUN IT. 
% If you press run, you have to close matlab to stop it.

computeAndStoreResult()



function computeAndStoreResult()

    % Initialize a variable to store the result
    result1 = 0;
    result2 = 0;
    
    % Create a timer object that executes the computation every 1 second
    t = timer('ExecutionMode', 'fixedRate', 'Period', 1, 'TimerFcn', @computeResult);
    t2 = timer('ExecutionMode', 'fixedRate', 'Period', 2, 'TimerFcn', @computeResult2);
    
    % Start the timer
    start(t);
    start(t2)

    
    
    % Display the final result
    disp(['Final result: ' num2str(result1)]);
    disp(['Final result: ' num2str(result2)]);

    function computeResult(~, ~)
        % Perform the computation (1 + 1) and store the result
        result1 = result1 + 1 + 1;
        disp(['Result after computation: ' num2str(result1)]);
    end

    function computeResult2(~, ~)
        % Perform the computation (1 + 1) and store the result
        result2 = result2 -2 ;
        disp(['Result after computation: ' num2str(result2)]);
    end

end
