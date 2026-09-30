function R = rot_x_rad(theta)
%ROT_X_RAD Rotation matrix about the positive x-axis.
%   R = ROT_X_RAD(THETA) returns a 3-by-3 rotation matrix. THETA is in
%   radians and may be a numeric or symbolic scalar.
%
%       R = [1    0           0;
%            0  cos(theta) -sin(theta);
%            0  sin(theta)  cos(theta)]

c = cos(theta);
s = sin(theta);

R = [1, 0,  0;
     0, c, -s;
     0, s,  c];
end
