function points = opt_cycle(original_vector, s1, s2)

    % This function is used to optimize the path obtained with the
    % navigation function algorithm
    % The number of segment is minimized considering as always a collision free path
    
    points = original_vector;
    index = 1;
    modif = 0;
    while 1
        if index+2 > length(points(:,1))
            if modif == 0
                break
            end
            index = 1;
            modif = 0;
        end

        p.x = linspace(points(index,1),points(index+2,1),1000);
        p.y = linspace(points(index,2),points(index+2,2),1000);

%         condition = (norm(points(index,:)-points(index+2,:)) == norm(points(index,:)-points(index+1,:)) + norm(points(index+1,:)-points(index+2,:)));
        
        % Collision check
        [in2,on2] = inpolygon(p.x, p.y, s1(1,:),s1(2,:));
        [in3,on3] = inpolygon(p.x, p.y, s2(1,:),s2(2,:));
        in = in2+in3;
        on = on2+on3;

        if (in == 0 & on == 0) %| (condition)
            points(index+1,:) = [];
            modif = modif + 1;
        else
            index = index + 1;
        end

    end
end