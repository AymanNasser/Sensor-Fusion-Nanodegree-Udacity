% Clustering the detected tracks into a single track for each target
function detectionClusters = clusterDetections(detections, vehicleSize)
  
  numOfDetections = numel(detections);
  distances = zeros(numOfDetections);
  
  for i=1:numOfDetections
    for j=i+1:numOfDetections
      % If the detections are from same sensor, then loop through every single 
      % detection point and measure the euclidean distance between all of them.  
      if detections{i}.SensorIndex == detections{j}.SensorIndex 
        distances(i,j) = norm(detections{i}.Measurement(1:2) - detections{j}.Measurement(1:2));
      else
        distances(i,j) = inf;
      end
    end
  end
  
  leftToCheck = 1:numOfDetections;
  i = 0;
  detectionClusters = cell(N,1);
  
  % 1- Pick the first detection in the check list and check for its clustering neighbors.
  % 2- If the distance between the first pick and remaining detections is less than 
  % the vehicle size, then group those detections and their respective radar 
  % sensor measurements, including range and velocity.
  % 3- For the group, take the mean of the range and velocity measurements.
  % 4- Create a new Cluster ID. Then, assign all the group detections to the same ID.
  % 5- Further, assign cluster, the mean range and velocity.
  % 6- In the end, delete from the list the detections which have already been assigned to a cluster.
  % 7- Keep repeating the process until the detection list is empty.
  while ~isempty(leftToCheck)
    
    underConsideration = leftToCheck(1);
    clusterInds = (distances(underConsideration,leftToCheck) < vehicleSize);
    detectionInds = leftToCheck(clusterInds);
    
    clusterDets = [detections{detectionInds}];
    clusterMeas = [clusterDets.Measurement];
    
    meas = mean(clusterMeas, 2);
    % Radar measurement vector has 6 values - where range and velocity for 
    % x and y coordinates reside at indices 1,2, 4, and 5:
    meas2D = [meas(1:2);meas(4:5)];
    
    i = i+1;
    detectionClusters{i} = detections{detectionInds(1)};
    detectionClusters{i}.Measurement = meas2D;
    % Deleting from the list the detection indices that have been assigned to a target
    leftToCheck(clusterInds) = [];
  end
  
endfunction

