function [ R, startPoint, stopPoint, centerPoint, turnAngle ] = circle_arc( lastWP, centerWP, nextWP, turnR )
% CIRICLE_ARC(lastWP, centerWP, nextWP, turnR) finds the center of the
% circle when using straight lines and circular arcs
%
%   This is based of the figure 10.8 in the navigation and control book by
%   Fossen
%
%   The WP has to be connected lastWP ---- centerWP --- nextWP
%   and turnR is the radius of the turning circle
%
%   it returns [ R, startPoint, stopPoint, centerPoint, turnAngle ] where:
%   R is R1 from figure 10.8 in Fossen
%   startPoint is the where the ship leaves the lines and start turning
%   stopPoint is where the ship stops turning and connect with next line
%   centerPoint is center point for the circle arc
%   turn angle is how many radians the ship needs to hold the turn

v = lastWP-centerWP; % working names to find the angle
u = nextWP-centerWP;

orthV = [v(2), -v(1)]; % this finds one of the 2 orthogonal vectors
if(dot(orthV, u) < 0)
    orthV = - orthV; % this choses the orthogonal vector point same way as u
end

a = acos(dot(u, v)/(norm(u)*norm(v)))/2;

R = turnR / tan(a); % equation 10.52 in mentioned book
startPoint = centerWP + (v / norm(v)) * R;
stopPoint = centerWP + (u / norm(u)) * R;
centerPoint = startPoint + turnR*(orthV/norm(orthV));
turnAngle = pi - 2*a;


end

