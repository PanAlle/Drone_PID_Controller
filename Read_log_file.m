file_name = 'Drone_PID_Controller.csv';

%Start read the csv ar row 1, so avoid the inital name of fitrst row
flight_data = csvread(file_name,1);

%Check for overshoot only in the case of a step function
if flight_data(2,5) == 1
overshoot = find(flight_data(:,2) > 1, 1); 
    % if overshoot vector is emty, then print no overshoot
    if isempty(overshoot) == true
        fprintf('No Overshoot \n');
    end
    %Compute settling time (+-5% to target)
    fprintf('Settling time %i', flight_data(find(flight_data(:,2) >= 0.95, 1), 1));
end

% Plot the value to a 2d plot
figure(1)
plot(flight_data(:,1), flight_data(:,2), flight_data(:,1),flight_data(:,5));

% Title based on the simulation choosen
if flight_data(2,5) == 1
    title('System step response')
else
    title('System trajectory following')
end
%% try to define the system to develop the PID in matlab
% -----------WIP---------------
%data = iddata(flight_data(:,2),flight_data(:,5), 0.01);
%sys = tfest(data, 4, 4)
%syspid = feedback(sys*pid(1,1,0),30)

