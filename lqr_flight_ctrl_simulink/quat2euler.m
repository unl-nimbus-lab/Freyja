%Convert Quaternion to Euler 
function [eul_ang] = quat2euler(q)

    %Check if vector and if have it has enough input values
    if ~isvector(q) || (length(q))~=4 
        error('Input to quat2rpy must be a vector and must have 4 values')
    end
    
    %Normailize Quaternion before using
    q=q/norm(q);
    
    %Roll
    eul_ang(1)      =   atan2(   (2*(q(1)*q(2)+(q(3)*q(4)))) , (q(1)^2+q(4)^2-q(2)^2-q(3)^2) );
    %Pitch 
    eul_ang(2)      =   asin(    2*((q(1)*q(3))-(q(2)*q(4))) );
    %Yaw
    eul_ang(3)      =   atan2(   (2*(q(1)*q(4)+(q(2)*q(3)))) , (q(1)^2+q(2)^2-q(3)^2-q(4)^2) );    
    
    %Equivalent built-in MATLAB function: quat2eul
end
