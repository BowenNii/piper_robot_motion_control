function R = rot_y(theta)
%ROT_Y_RAD Rotation matrix about the positive y-axis.
%   R = ROT_Y_RAD(THETA) returns a 3-by-3 rotation matrix. THETA is in
%   radians and may be a numeric or symbolic scalar.
%
%       R = [ cos(theta)  0  sin(theta);
%                 0       1       0;
%             -sin(theta)  0  cos(theta)]

c = cos(theta);
s = sin(theta);

R = [ c, 0, s;
      0, 1, 0;
     -s, 0, c];
end
