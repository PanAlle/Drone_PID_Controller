file_name = 'Drone_PID_Controller.csv'

%Start read the csv ar row 1, so avoid the inital name of fitrst row
flight_data = csvread(file_name,1);

figure(1)
plot(flight_data(:,1), flight_data(:,2), flight_data(:,1),flight_data(:,5))



data = iddata(flight_data(:,2),flight_data(:,5), 0.01);

%sys = tfest(data, 4, 4)
%syspid = feedback(sys*pid(1,1,0),30)

