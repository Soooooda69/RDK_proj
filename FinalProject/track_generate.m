% Load the image
image = imread(['kirby.jpg']);

% Convert the image to grayscale if it's not already
if ndims(image) == 3
    image = rgb2gray(image);
end

% Display the original image
figure(1);
imshow(image);
title('Original Image');

% Perform some basic image processing
% In this case, we'll just apply a median filter
processed_image = medfilt2(image);

% Display the processed image
figure(2);
imshow(processed_image);
title('Processed Image');

% Detect the edges in the image
edges = edge(processed_image, 'Canny');

% Display the edges
figure(3);
imshow(edges);
title('Detected Edges');

% Thin the edges to a single pixel width using the 'thin' operation
thin_edges = bwmorph(edges, 'thin', Inf);

% Display the thinned edges
figure(4);
imshow(thin_edges);
title('Thinned Edges');

% Generate the robot's path (track) from the thinned edges
% Note: You may need to adjust this to fit your specific requirements for a "track"
% In this simple example, we'll just find the row and column indices of the edge pixels
[thin_edge_rows, thin_edge_cols] = find(thin_edges == 1);

% Now, we'll create a path (track) for the robot to follow
% We'll just have the robot visit each edge pixel in the order they were found
track = [thin_edge_rows, thin_edge_cols];

% Display the track
figure(5);
plot(track(:,2), track(:,1), 'r.');  % Plotting (col, row) because plot uses (x, y)
title('Robot Track');
axis ij;  % Make the y-axis point downwards like in an image
