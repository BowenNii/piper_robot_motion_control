function T = mdh_transform(alpha_prev,a_prev,d_i,theta_i)
%MDH_TRANSFORM 改进DH（Craig/《现代机器人学》编号）单节变换。
%   输入：alpha_prev [rad]、a_prev [m]、d_i [m]、theta_i [rad]。
%   输出：上一连杆坐标系到当前连杆坐标系的4×4变换。
%   公式：T_(i-1,i)=RotX(alpha_(i-1))*TransX(a_(i-1))* ...
%                    RotZ(theta_i)*TransZ(d_i)。
%   注意：不要与标准DH的RotZ*TransZ*TransX*RotX次序混用。
values=[alpha_prev,a_prev,d_i,theta_i];
assert(all(isfinite(values)) && numel(values)==4, 'DH参数必须是有限标量');
ca=cos(alpha_prev); sa=sin(alpha_prev);
ct=cos(theta_i); st=sin(theta_i);
T=[ct,-st,0,a_prev;
   ca*st,ca*ct,-sa,-d_i*sa;
   sa*st,sa*ct,ca,d_i*ca;
   0,0,0,1];
end
