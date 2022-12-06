function S = skew_f(v)

    % This function computes the skew-symmetric matrix for a vector v.
    
    S = [0 -v(3) v(2);
         v(3) 0 -v(1);
         -v(2) v(1) 0];
     
end