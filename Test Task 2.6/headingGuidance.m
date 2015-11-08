function [ guidance ] = headingGuidance( position )



global k WP_xpos WP_ypos R_k1 K_p_guidance;

N = length(WP_ypos); %number of way points

if k == 0 %not yet reached first way point
    xk = 0;
    yk = 0;
else
    xk = WP_xpos(k);
    yk = WP_ypos(k);
end

xk1 = WP_xpos(k+1);
yk1 = WP_ypos(k+1);

x_p = atan2(yk1 - yk,xk1 - xk); %(10.64)
e = - (position(1) - xk)*sin(x_p) + (position(2)- yk)*cos(x_p); %(10.59)

K_p_guidance = 1/sqrt(R_k1^2-e^2);

X_r = atan(-K_p_guidance*e);
guidance = [x_p; X_r];
    
circle_of_acceptance = (xk1 - position(1))^2 + (yk1 - position(2))^2;
if circle_of_acceptance <= R_k1^2 %go to next waypoint (10.86)
    if k < N-1
        k = k+1;
    end
end
end

