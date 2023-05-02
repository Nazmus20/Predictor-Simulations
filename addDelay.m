function [delayedMtx] = addDelay(time_vector, undelayedMtx, delayAmount,...
    initial_value)

% addDelay.m
%
% Add a specified delay to a vector of data

%%%INPUTS%%%
% time_vector: A time vector consisting of time values of some data points,
% seconds
% undelayedMtx: A matrix of datasets that is not delayed. Each row of the
% matrix corresponds to a vector of data points. Each column of the data
% points corresponds to each element of the time_vector. As such, the
% column length of undelayedMtx and the length of time_vector must be the
% same. The undelayedMtx is at the the time instant t, undelayedMtx(t).
% delayAmount: Specify how much delay should be added, seconds. Must be
% positive definite, delayAmount >= 0.
% initial_value: The initial values of the delayedMtx to replace the empty
% elements shifted by the specified time delay. Must have the same number
% of rows as the undelayedMtx

%%%OUTPUTS%%%
% delayedMtx: Elements of the undelayedMtx shifted by the delayAmount.
% The delayedMtx is at the time instant t+delayAmount, undelayedMtx( t +
% delayedAmount).

%Obtain the length of the time vector
length_of_time = length(time_vector);
%Obtain the size of the time undelayed matrix
[row_of_mtx, col_of_mtx] = size(undelayedMtx);
%Obtain the row of the initial_value
[row_of_init_val, ~]  = size(initial_value);

if col_of_mtx ~= length_of_time | row_of_mtx ~= row_of_init_val
    disp('Time vector and the corresponding input data have a mismatch.')  
    disp(strcat('The time vector should be a 1xN vector and the input', ...
        'matrix must be an MXN matrix.'))
    disp('AND The initial value vector must be a column vector.')
    
else
    %Find the first index in time vector that passes the delay time
    del_idx = find(time_vector - delayAmount > 0, 1);
    for i = 1:length_of_time
        if time_vector(i) <= delayAmount
            delayedMtx(:,i) = initial_value;
        else
            delayedMtx(:,i) = undelayedMtx(:,i+1-del_idx);
        end
    end
end