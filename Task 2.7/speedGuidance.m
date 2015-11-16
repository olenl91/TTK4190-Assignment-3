function [ output ] = speedGuidance( shippos, target_pos, X_t )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global U_t Kappa delta_speed desired_distance;

position_error = shippos - target_pos;
distance_error = sqrt(position_error' * position_error);

s = [cos(X_t), sin(X_t)]*position_error + desired_distance; %lateral distance (10.10)
%e = [sin(X_t), cos(X_t)]*distance_error; %cross-track distance (10.10)

U_d = U_t - Kappa*s/sqrt(s^2 + delta_speed^2);%(10.15)

output = [U_d; distance_error];
end

