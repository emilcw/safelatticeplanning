function trajectory = load_trajectory( fileName, header_rows )

data = dlmread(fileName,'',header_rows,0);
trajectory.t = data(:,1);

trajectory.x = data(:,2);
trajectory.y = data(:,3);
trajectory.z = data(:,4);

trajectory.vx = data(:,5);
trajectory.vy = data(:,6);
trajectory.vz = data(:,7);


trajectory.roll = data(:,8);
trajectory.pitch = data(:,9);
trajectory.yaw = data(:,10);

trajectory.thrust = data(:,11);

end

