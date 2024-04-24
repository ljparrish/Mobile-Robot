%% Create Car
function [newXCar, newYCar] = GenerateRobot(x_car, y_car, l_car, w_car, angle)
%This function will create a rectangle that represents a car in the plot

%Defines rotation matrix that accepts angle input
rotationMatrix = [cos(angle), -sin(angle) ; sin(angle), cos(angle)];
%Defines the rectangle the origin which we will use to calculate new
%rotated points
baseXCar = [(0 - (l_car/2)), (0 + (l_car/2)), (0 + (l_car/2)), (0 - (l_car/2))];
baseYCar = [(0 - (w_car/2)), (0 - (w_car/2)), (0 + (w_car/2)), (0 + (w_car/2))];



%Uses the origin rectangle and separates the x and y components into column
%vector
basePoint1 = [baseXCar(1); baseYCar(1)];
basePoint2 = [baseXCar(2); baseYCar(2)];
basePoint3 = [baseXCar(3); baseYCar(3)];
basePoint4 = [baseXCar(4); baseYCar(4)];


%Uses rotation matrix to get the new points of the rectangle after rotation
newPoint1 = rotationMatrix * basePoint1;
newPoint2 = rotationMatrix * basePoint2;
newPoint3 = rotationMatrix * basePoint3;
newPoint4 = rotationMatrix * basePoint4;

%Calculates the vector length between the new point and the original point
%pre rotation
vector1 = newPoint1 - basePoint1;
vector2 = newPoint2 - basePoint2;
vector3 = newPoint3 - basePoint3;
vector4 = newPoint4 - basePoint4;

%Creates actual car at the actual center of the rectangle
xcar = [(x_car - (l_car/2)), (x_car + (l_car/2)), (x_car + (l_car/2)), (x_car - (l_car/2))];
ycar = [(y_car - (w_car/2)), (y_car - (w_car/2)), (y_car + (w_car/2)), (y_car + (w_car/2))];

%Now creates a column vector based off the actual coordinates of the actual
%rectangle we are trying to rotate
point1 = [xcar(1); ycar(1)];
point2 = [xcar(2); ycar(2)];
point3 = [xcar(3); ycar(3)];
point4 = [xcar(4); ycar(4)];

%Calculates actual rotated coordinates by adding the calculated vectors to
%the original points
rotatedPoint1 = point1 + vector1;
rotatedPoint2 = point2 + vector2;
rotatedPoint3 = point3 + vector3;
rotatedPoint4 = point4 + vector4;

%Creates a new x and y array based off the new rotated points
newXCar = [rotatedPoint1(1,:), rotatedPoint2(1,:), rotatedPoint3(1,:), rotatedPoint4(1,:)];
newYCar = [rotatedPoint1(2,1), rotatedPoint2(2,1), rotatedPoint3(2,1), rotatedPoint4(2,1)];

%Plots the car with the x and y coordinates and inputted color
%fill(newXCar, newYCar, color, 'LineStyle','none');
end