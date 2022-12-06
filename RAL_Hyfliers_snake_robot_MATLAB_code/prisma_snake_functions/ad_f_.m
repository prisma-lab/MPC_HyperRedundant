function ad = ad_f_(V)

    % This function computes the adjoint of a vector
    
    ad = [skew_f(V(1:3)) zeros(3,3);
          skew_f(V(4:6)) skew_f(V(1:3))];

end