% 2DX3 - Deliverable (2) - Final Project
% MATLAB Data Visualization Script
% Matthew Jarzynowski

% Read the data from the file
data = readmatrix("data.txt", 'Delimiter', ',');

% Extract X, Y, and Z coordinates
X = data(:, 1);
Y = data(:, 2);
Z = data(:, 3);

% Create a 3D scatter plot
scatter3(X, Y, Z, 'filled');
hold on;

% Determine the number of complete sets of 128 points
numSets = floor(length(X) / 128);
pointsPerSet = 128;

numLayers = numSets - 1;

% Connect each set of 128 points to each other and add transparent surfaces
for setIdx = 1:numSets
    
    % Draw lines within the set
    startIndex = (setIdx - 1) * pointsPerSet + 1;
    endIndex = startIndex + pointsPerSet - 1; % Last point in the set
    
    % Connect points within the same rotation
    plot3(X(startIndex:endIndex), Y(startIndex:endIndex), Z(startIndex:endIndex), '-o', ...
          'MarkerSize', 2, 'MarkerFaceColor', 'b');

    % Connect the last point to the first point to complete the circle
    plot3([X(endIndex), X(startIndex)], [Y(endIndex), Y(startIndex)], ...
          [Z(endIndex), Z(startIndex)], '-o', 'MarkerSize', 2, 'MarkerFaceColor', 'b');
end

% Connect the corresponding points between rotations
for pointIdx = 1:pointsPerSet
    for setIdx = 1:numSets-1
        startIndex = (setIdx - 1) * pointsPerSet + pointIdx;
        nextStartIndex = startIndex + pointsPerSet;

        % Draw a blue line between the corresponding points of subsequent rotations
        plot3([X(startIndex), X(nextStartIndex)], [Y(startIndex), Y(nextStartIndex)], ...
              [Z(startIndex), Z(nextStartIndex)], 'k', 'LineWidth', 0.5);
    end
end

% Add transparent surfaces between the layers
for layerIdx = 1:numLayers
    for pointIdx = 1:pointsPerSet
        currentBaseIdx = (layerIdx - 1) * pointsPerSet + pointIdx;
        nextBaseIdx = currentBaseIdx + pointsPerSet;

        % Handle the case where the point is the last one in the layer
        if pointIdx == pointsPerSet
            % Connect the last point to the first point in the same layer
            nextPointIdx = (layerIdx - 1) * pointsPerSet + 1;
        else
            % Proceed as normal
            nextPointIdx = currentBaseIdx + 1;
        end
        
        % Define vertices for the two polygons (triangles) to form a rectangle
        vertices = [X(currentBaseIdx) Y(currentBaseIdx) Z(currentBaseIdx);
                    X(nextPointIdx) Y(nextPointIdx) Z(nextPointIdx);
                    X(nextBaseIdx) Y(nextBaseIdx) Z(nextBaseIdx)];

        if pointIdx < pointsPerSet
            % Add the next point in the subsequent layer for the second triangle
            vertices = [vertices; X(nextBaseIdx + 1) Y(nextBaseIdx + 1) Z(nextBaseIdx + 1)];
        else
            % Connect to the first point in the subsequent layer to close the loop
            vertices = [vertices; X(layerIdx * pointsPerSet + 1) Y(layerIdx * pointsPerSet + 1) Z(layerIdx * pointsPerSet + 1)];
        end
        
        % Draw two triangles to form the surface between the layers
        fill3(vertices([1,2,3],1), vertices([1,2,3],2), vertices([1,2,3],3), 'b', 'FaceAlpha', 0.35);
        
        % Only add the second triangle if we're not at the end of the layer
        if pointIdx < pointsPerSet
            fill3(vertices([2,3,4],1), vertices([2,3,4],2), vertices([2,3,4],3), 'b', 'FaceAlpha', 0.35);
        end
    end
end


% Finalize the plot
title({'2DX3 - Deliverable (2) - Final Project - Box Demo','Matthew Jarzynowski - 400455803'});

xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
grid on; % Add a grid for better visibility
hold off;

disp('Plotting complete.'); % Information

% Video Rendering
% ----------------------------

% Assuming your 3D figure is already created and is the current figure

% Set the size of the figure window
%figureHandle = gcf;
%set(figureHandle, 'Position', [100, 100, 1200, 900]); % Adjust as needed

% Set up the video writer
%videoFileName = 'nice.avi';
%videoWriter = VideoWriter(videoFileName);
%open(videoWriter);

% Set the number of degrees per frame for a slower rotation
%degreesPerFrame = 1;  % Half of the 3 degrees per frame originally specified
%numFrames = 360 / degreesPerFrame; % This will result in one full rotation

% Find the center of the plot box and set axes limits to maintain the same size
%xLimits = get(gca, 'XLim');
%yLimits = get(gca, 'YLim');
%zLimits = get(gca, 'ZLim');
%center = [mean(xLimits), mean(yLimits), mean(zLimits)];

% Lock the aspect ratio and the limits of the axes
%axis vis3d manual;
%set(gca, 'XLim', xLimits, 'YLim', yLimits, 'ZLim', zLimits);

% Lock the camera position to avoid the 'bouncing' effect
%set(gca, 'CameraViewAngleMode', 'manual', ...
  %       'CameraPositionMode', 'manual', ...
 %        'CameraTargetMode', 'manual');

% Rotate the figure and capture frames
%for angle = 0:degreesPerFrame:(numFrames * degreesPerFrame - degreesPerFrame)
    % Rotate the camera around the z-axis
    %camorbit(degreesPerFrame, 0, 'data', [0 0 1]);
    
    % Set the camera target to the center of the plot box
   % camtarget(center);
    
    % Capture the frame
  %  frame = getframe(figureHandle);
    
    % Write the frame to the video
 %   writeVideo(videoWriter, frame);
%end

% Close the video writer
%close(videoWriter);
%disp(['Video saved as ', videoFileName]);


