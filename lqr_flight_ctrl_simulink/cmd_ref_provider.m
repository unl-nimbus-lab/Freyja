function URef = cmd_ref_provider( t )

  persistent w alpha beta H
  
  w = 1;
  alpha = 1.5;
  beta = 0.75;
  H = 5;
  
  URef = [ -alpha*w*w/4 * cos( w*t/2 );
           -beta*w*w * sin( w*t );
           0;
           0 ];
end