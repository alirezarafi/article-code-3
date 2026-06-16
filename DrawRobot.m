function [] = DrawRobot(robot_radius, robot_x, robot_y, headingAngle, colour)
    %% Generate Circle
    theta    = linspace(0, 2*pi, 360);
    circle_x = 2*robot_radius * cos(theta);
    circle_y = 2*robot_radius * sin(theta);
    
    %% Calculate end points of the line
    % Line is 0.02m larger than the robot
    lineEnd_x = cos(headingAngle) * (1.5*robot_radius + 0.25);
    lineEnd_y = sin(headingAngle) * (1.5*robot_radius + 0.25);
    
    %% Draw robot
    % Draw circle around current location of robot with مشخص شده رنگ
    fill((circle_x + robot_x), (circle_y + robot_y), colour, 'FaceAlpha', 0.25); % استفاده از 'FaceAlpha' برای شفافیت (اختیاری)
    
    % Draw line from center of robot in the direction of the current heading
    line([robot_x, (lineEnd_x + robot_x)], [robot_y, (lineEnd_y + robot_y)], 'color', 'k','LineWidth', 3);
end
