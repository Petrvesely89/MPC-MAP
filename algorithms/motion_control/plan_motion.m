function [public_vars] = plan_motion(read_only_vars, public_vars)
%PLAN_MOTION Summary of this function goes here
XR = public_vars.estimated_pose(1);
YR = public_vars.estimated_pose(2);
thetaR = public_vars.estimated_pose(3);

epsilon = 0.2;
k = 1;

% I. Pick navigation target
[target, public_vars.path, finished] = get_target(public_vars.estimated_pose, public_vars.path);

if finished || isempty(target)
    public_vars.motion_vector = [0, 0];
    return;
end
% II. Compute motion vector
xP = XR + epsilon*cos(thetaR);
yP = YR + epsilon*sin(thetaR);

dxP = k * (target(1) - xP);
dyP = k * (target(2) - yP);

v = dxP*cos(thetaR) + dyP*sin(thetaR);
u = (1/epsilon) * (-dxP*sin(thetaR) + dyP*cos(thetaR));

public_vars.motion_vector = kinematics(v, u);

end

function [wheel] = kinematics(v,u)
wheelL=(2*v+u)/2;
wheelR=(2*v-u)/2;
wheel=[wheelL,wheelR];
end