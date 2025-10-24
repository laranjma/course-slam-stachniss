function [vector] = t2v(transform)

x = transform(1, 3);
y = transform(2, 3);
theta = acos(transform(1, 1));

vector = [x; y; theta];

end