function [ output ] = guidance( position )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
global e_d s_d U_t delta_s delta_e chi_t kappa p_t R_p;

p_t = p_t + [sin(53.69*pi/180)*U_t; cos(53.69*pi/180)*U_t];
R =  R_p'*(position - p_t);
e = e_d + R(2); %(10.10)
s = s_d + R(1);

chi_r = atan(-e/delta_e);
chi_d = chi_t + chi_r;
Kappa = 7*norm(position-p_t)/sqrt((position-p_t)'*(position-p_t)+9000^2);

U_d = U_t - Kappa*s/(sqrt(s^2+delta_s^2));
output = [chi_d, U_d];
end

