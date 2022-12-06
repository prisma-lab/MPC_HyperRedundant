function y = get_circle(cc,r)
    
    % This function computes the points (x,y) of a circumference having cc as
    % center and r as radius
    
    alfa = 0:0.01:2*pi;
    y = cc(2) + r*sin(alfa);
    x = cc(1) + r*cos(alfa);
    y = [x;y];
    
end