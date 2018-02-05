function rpy = rot2rpy( R )

  rpy(1) = atan( R(3,2)/R(3,3) );
  rpy(2) = atan( -R(3,1)/sqrt( R(3,2)^2 + R(3,3)^2 ) );
  rpy(3) = atan( R(2,1)/R(1,1) );
end