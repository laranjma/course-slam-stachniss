% Computes the total error of the graph
function Fx = compute_global_error(g)

Fx = 0;

% Loop over all edges
for eid = 1:length(g.edges)
  edge = g.edges(eid);

  % pose-pose constraint
  if (strcmp(edge.type, 'P') != 0)

    X1 = v2t(g.x(edge.fromIdx:edge.fromIdx+2));  % the first robot pose
    X2 = v2t(g.x(edge.toIdx:edge.toIdx+2));      % the second robot pose

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    z = edge.measurement; % measurement as seen from x1
    Z = v2t(z);
    E = inv(Z)*(inv(X1)*X2);
    e = t2v(E);
    Omega = edge.information; 
    Fx += e'*Omega*e;

  % pose-landmark constraint
  elseif (strcmp(edge.type, 'L') != 0)
    x = g.x(edge.fromIdx:edge.fromIdx+2);  % the robot pose
    l = g.x(edge.toIdx:edge.toIdx+1);      % the landmark

    %TODO compute the error of the constraint and add it to Fx.
    % Use edge.measurement and edge.information to access the
    % measurement and the information matrix respectively.
    z = edge.measurement;
    X = v2t(x);
    Ri = X(1:2,1:2);
    e = Ri'*(l(1:2) - x(1:2)) - z;
    Omega = edge.information; 
    Fx += e'*Omega*e;
  end

end
