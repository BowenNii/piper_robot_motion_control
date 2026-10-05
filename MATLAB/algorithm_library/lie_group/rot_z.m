function R = rot_z(theta)
%ROT_Z_RAD Rotation matrix about the positive z-axis.
%   R = ROT_Z_RAD(THETA) returns a 3-by-3 rotation matrix. THETA is in
%   radians and may be a numeric or symbolic scalar.
%
%       R = [cos(theta) -sin(theta)  0;
%            sin(theta)  cos(theta)  0;
%                 0           0      1]

c = cos(theta);
s = sin(theta);

R = [c, -s, 0;
     s,  c, 0;
     0,  0, 1];
end
