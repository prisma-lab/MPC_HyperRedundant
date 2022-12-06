function y = get_polygon(cc,l,h)

    % Vertices of the poligon (we suppose to have only rectangular o square obstacles, so with 4 vertices)
    v1 = [cc(1)-l/2, cc(2)-h/2];
    v2 = [cc(1)+l/2, cc(2)-h/2];
    v3 = [cc(1)+l/2, cc(2)+h/2];
    v4 = [cc(1)-l/2, cc(2)+h/2];
    
    % From them we get the segment of each side of the polygon
    % The numer of point of each segment can be increased (but the elaboration time increases)
    l1(1,:) = linspace(v1(1),v2(1),200*l);
    l1(2,:) = linspace(v1(2),v2(2),200*l);
    l2(1,:) = linspace(v2(1),v3(1),200*h);
    l2(2,:) = linspace(v2(2),v3(2),200*h);
    l3(1,:) = linspace(v3(1),v4(1),200*l);
    l3(2,:) = linspace(v3(2),v4(2),200*l);
    l4(1,:) = linspace(v4(1),v1(1),200*h);
    l4(2,:) = linspace(v4(2),v1(2),200*h);

    % The connection of all this point produces the polygon
    y = cat(2,l1,l2,l3,l4);
    
end