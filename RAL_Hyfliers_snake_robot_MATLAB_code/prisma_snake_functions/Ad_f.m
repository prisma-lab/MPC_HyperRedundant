function Ad = Ad_f(M)
    
    % This function computes the Adjoint of the inverse of the M matrix. 
    % Remember to compute inv(M) if necessary (Step 1)

    Ad = [M(1:3,1:3) zeros(3,3);
          skew_f(M(1:3,4))*M(1:3,1:3) M(1:3,1:3)];

end