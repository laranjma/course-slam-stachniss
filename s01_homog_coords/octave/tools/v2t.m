function [transform] = v2t(vector)

x = vector(1);
y = vector(2);
theta = vector(3);

transform = [cos(theta) -sin(theta) x;
             sin(theta)  cos(theta) y;
             0           0          1];

end
