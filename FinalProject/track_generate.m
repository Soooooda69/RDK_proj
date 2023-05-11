% Load the image
image = imread('kirby.jpg');

% Define the scale (in cm/pixel)
scale = 0.01;  % Adjust this to your actual scale

% Define the maximum size (in cm)
max_size_cm = 20;

% Calculate the maximum size in pixels
max_size_pixels = max_size_cm / scale;

% Crop the image to the maximum size
image = image(1:min(max_size_pixels, size(image, 1)), 1:min(max_size_pixels, size(image, 2)));

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

% Thin the edges to a single pixel width using the 'thin' operation
thin_edges = bwmorph(edges, 'thin', Inf);

% Display the thinned edges
figure(3);
imshow(thin_edges);
title('Thinned Edges');

% Generate the robot's path (track) from the thinned edges
[thin_edge_rows, thin_edge_cols] = find(thin_edges == 1);

% Now, we'll create a path (track) for the robot to follow
% We'll just have the robot visit each edge pixel in the order they were found
track = [thin_edge_rows, thin_edge_cols];

% Define the downsampling factor
downsampling_factor = 2.2;  % Adjust this to your desired level of downsampling

% Downsample the track
downsampled_track = track(1:downsampling_factor:end, :);

% Display the original and downsampled tracks
figure(4);
plot(track(:,2), track(:,1), 'r.');  % Plotting (col, row) because plot uses (x, y)
title('Original Track');
axis ij;  % Make the y-axis point downwards like in an image

figure(5);
plot(downsampled_track(:,2), downsampled_track(:,1), 'b');  % Plotting (col, row) because plot uses (x, y)
title('Downsampled Track');
axis ij;  % Make the y-axis point downwards like in an image

a = downsampled_track;
index = []
i = 1
track = zeros(length(a),2);
%Sort by distance between points
for i = 1:length(track)-1
    d = (a(:,1)-track(i,1)).^2+(a(:,2)-track(i,2)).^2;

    [mind,index] = min(d);
    track(i+1,:) = a(index,:);
    a(index,:) = [];

end
index2 = find(track(:,2)==0);
track(index2,:) = [];

plot(track(:,1),track(:,2),'k.')
dis = [];
%Remove redundant points by point-to-point distance
data = track;
i = 1
a = zeros(length(data),2);
while length(data)>0
    a(i,:) = data(1,:);
    index = find((data(:,1)-a(i,1)).^2+(data(:,2)-a(i,2)).^2<30);
    data(index,:) = [];
    i = i+1;
end
index2 = find(a(:,2)==0);
a(index2,:) = [];
plot(a(:,1),a(:,2),'k.')
track = a;
plot(track(:,1),track(:,2),'k')

%skip long solid lines
for i = 1:length(track)-1
    dis(i) = (track(i+1,1)-track(i,1)).^2+(track(i+1,2)-track(i,2)).^2;
end
dis = dis';
n = find(dis>1800);
figure
plot(track(1:n(1)-1),track(1:n(1)-1,2),'k')
hold on
for i = 1:length(n)-1
    plot(track(n(i)+1:n(i+1)),track(n(i)+1:n(i+1),2),'k')
    hold on
end
plot(track(n(end)+1:length(track),1),track(n(end)+1:length(track),2),'k')
save("track.mat","track","n")


% Downsample the track
downsampled_track = track(1:downsampling_factor:end, :);

% Save the downsampled track to a .mat file in your local directory
save('Kirby.mat', 'downsampled_track');