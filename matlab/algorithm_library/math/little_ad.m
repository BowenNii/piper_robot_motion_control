function ad = little_ad(xi)
%LITTLE_AD 速度旋量的小伴随；xi=[omega;v]。
W = skew(xi(1:3));
V = skew(xi(4:6));
ad = [W, zeros(3,3);
      V, W];
end
