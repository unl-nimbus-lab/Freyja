function traj_ref = trajectory_provider( t )

  persistent w alpha beta H
  w = 2;
  alpha = 1.5;
  beta = 0.75;
  H = 5;
  
  if( t < 16.0 )
  traj_ref = [ alpha * cos( w*t );
               beta * sin( w*t );
               -2;%-H + sin( w*t );
               -alpha * w * sin( w*t );
               beta * w * cos( w*t );
               0; %w * cos( w*t )
             ];
  else if( t < 14 )
      traj_ref = zeros(6,1);
    else
      traj_ref = [ beta * sin( w*t );
                 alpha * cos( w*t );
                 -2;%-H + sin( w*t );
                 beta * w * cos( w*t );
                 -alpha * w * sin( w*t );
                 0;%w * cos( w*t )
                 ];
    end
  end
%  traj_ref = zeros(6,1);
  %traj_ref(7) = 0;
  
end