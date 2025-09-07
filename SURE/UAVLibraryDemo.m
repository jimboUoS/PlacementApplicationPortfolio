% Experimenting with the UAV toolbox



coords= [      0           0 
     -117.75     -262.24 
     -48.027     -305.49 
      33.137       -79.7 
      217.83     -145.47 
      122.98     -180.45 
      206.49     -19.773 
      303.45     -156.63 
      55.716     -270.17 
      125.43      58.647 
      52.152      114.88 
           0           0];
coords=[coords 30*ones(12,1)];

vels = repmat([12 0 0],12,1);

times = cumsum(5*ones(12,1));


T=fixedwingFlightTrajectory(coords,vels,times);
show(T);