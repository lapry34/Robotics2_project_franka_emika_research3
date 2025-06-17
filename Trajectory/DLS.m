function J_inv_damped = DLS(J)
   % Simple DLS with hardcoded values
   
   sigma_min = svds(J, 1, 'smallest');
   
   if sigma_min >= 1e-3
       lambda = 0;  % use pinv
   else
       lambda = 0.01 * (1 - sigma_min/1e-3);
   end
   
   if lambda == 0
       J_inv_damped = pinv(J);
   else
       J_inv_damped = J' / (J * J' + lambda^2 * eye(size(J, 1)));
   end
end