function [traj_planned, qd, points, points_path] = navigation_function(q0,qg,cc, box_len ,box_len_aug,  r1_aug, r2_aug)
    
    % This function cumputes the collision-free path in a known environment
    % It is based on the artificial potential method
    % The map is divided in cells and to each cell is associated a value of
    % potential
    % The obstacles are ignored assigning them a high potential
    % The path is built in term of cells with lower potentials
    
    % Initial configuration
    q_k = q0;

    % Polygon associated to each obstacles 
    s1 = get_circle(cc(2,:),r1_aug); 
    s2 = get_circle(cc(3,:),r2_aug); 
    
    % Definition of the bounds of the map
    box = get_polygon(cc(1,:),box_len,box_len);
    box_n = get_polygon(cc(1,:),box_len_aug,box_len_aug);

    A = [cc(1,1)-box_len/2,cc(1,2)-box_len/2];                              % Vertix of the arena

    % Length of an elementary cell (used as resolution)
    d = 0.05;                                                              
    
    %initialization of the grid
    M = ones(box_len/d,box_len/d);                                          % Matrix of potentials
    already_visited = zeros(box_len/d,box_len/d);                           % Matrix of boolean (element = 1 cell visited, element = 0 cell unvisited)
    
    max_potential_obs = (box_len * 2) / d;                                  % Choosing the potential associated to the obstacle in function of the resolution

    center_vec = [];
    found = false;
    
    % Loop to create the grid-map
    for j=0:1:box_len/d-1
        for i=0:1:box_len/d-1

            % Center of the cell
            center_vec_temp = [A(1)+d/2+d*i,A(2)+d/2+d*j];                  

            cell = get_polygon(center_vec_temp,d,d);

            % Check if a point on the cell is on or inside the obstacles
            [in2,on2] = inpolygon(cell(1,:),cell(2,:), s1(1,:),s1(2,:));
            [in3,on3] = inpolygon(cell(1,:),cell(2,:), s2(1,:),s2(2,:));
            in = in2+in3;
            on = on2+on3;

            [in4,on4] = inpolygon(cell(1,:),cell(2,:), box(1,:), box(2,:));
            [in5,on5] = inpolygon(cell(1,:),cell(2,:), box_n(1,:), box_n(2,:));

            if (in == 0 & on == 0) & (in5 ==1 & on4 == 0 | on5 == 1)        % If the cell belongs to C-free
                
                %Uncomment if you want to plot the map step by step
%                 rectangle('Position',[A+[d*i,d*j], d, d], 'LineStyle', '-','EdgeColor','k') % Plot it on the workspace   
                M(length(M)-j,1+i) = -1;                                    % Give to it the potential -1

                % Searching for the goal inside the grid-map 
                % It is the starting point for the algorithm
                temp = fix(center_vec_temp * 1000) / 1000;
                
                % Distance between the goal and the center of a cell
                dist = sqrt((temp(1) - qg(1))^2 + (temp(2) - qg(2))^2);     
                
                if(dist <= d/2 * sqrt(2) && found == false)                 % If the distance is lower than the distance between the center and the vertix of the cell, the point is inside it
                    
                    M(length(M)-j,1+i) = 0;                                 % The associated potential is zero
                    already_visited(length(M)-j,i+1) = 1;                   % The cell is marked as already visited
                    final_indices = [length(M)-j,1+i];                      % The indices of that cell is saved
                    found = true;                                           % Avoid to repeat this piece of code if the goal cell is already found
                end

                center_vec = cat(1,center_vec,center_vec_temp);             % Vector of centers of each cells of C-free
            else
                M(length(M)-j,1+i) = max_potential_obs;                     % Max potential is associated to the cell belonging CO

            end
            % Uncomment if you want to plot the map step by step
%               pause(0.01)
        end
    end

    % Exploring the grid
    for i=1:length(M)
        for j=1:length(M)
            if fix(M(i,j)) == max_potential_obs
                    already_visited(i,j) = 1;                               % Mark as visited each cell where an obstacle was found
            end
        end
    end
    
    pot = 1;                                                                 
    done = false;
    
    % Assigning potentials to the whole map
    while done == false
        
        now_visited = [];       
        
        % Serch for the already visited cells with the lower potential
        for z=1:length(M)
            for s=1:length(M)
                if(already_visited(z,s) == 1 && min(min(M + pot)) == M(z,s))
                    now_visited = [now_visited; [z,s]];                     % A vector is created with the indices of the visited cells with lower potential
                end
            end
        end
        
        % The map is explored choosing an adjacency law
        if(~isequal(already_visited, ones(size(already_visited))))          % If there are unvisited cells, the exploration can start
            adjacent_cells = [];
            for i=1:length(now_visited(:,1))                                % For each cell to analyze, try to explore its adjacent cells

                if(now_visited(i,2)+1<=length(M))                           % If the cell above belongs to M
                    up = [now_visited(i,1),now_visited(i,2)+1];
                    if already_visited(up(1),up(2)) == 0                    % And if this cell is unvisited 
                        adjacent_cells = [adjacent_cells; up];              % Add it as a cell to explore in the adjacent_cell vector               
                    end
                end

                if(now_visited(i,2)-1>0)                                    % If the cell below belongs to M
                    down = [now_visited(i,1),now_visited(i,2)-1];           
                    if already_visited(down(1),down(2)) == 0                % And if this cell is unvisited 
                        adjacent_cells = [adjacent_cells; down];            % Add it as a cell to explore in the adjacent_cell vector
                    end
                end

                if(now_visited(i,1)-1>0)                                    % If the cell on the left belongs to M
                    left = [now_visited(i,1)-1,now_visited(i,2)];
                    if already_visited(left(1),left(2)) == 0                % And if this cell is unvisited
                        adjacent_cells = [adjacent_cells; left];            % Add it as a cell to explore in the adjacent_cell vector
                    end
                end

                if(now_visited(i,1)+1<=length(M))                           % If the cell on the right belongs to M
                    right = [now_visited(i,1)+1,now_visited(i,2)];
                    if already_visited(right(1),right(2)) == 0              % And if this cell is unvisited
                        adjacent_cells = [adjacent_cells; right];           % Add it as a cell to explore in the adjacent_cell vector
                    end
                end

            end
            
            % Now the adjacent_vector with all the cell to visit is ready
            for k=1:1:length(adjacent_cells(:,1))
                M(adjacent_cells(k,1),adjacent_cells(k,2)) = pot;               % The adjacent_cell vector is composed by the index of M, so i use it to modify the potential of the cell visited in this moment
                already_visited(adjacent_cells(k,1),adjacent_cells(k,2)) = 1;   % All of these cells are marked now as visited
            end
            pot = pot + 1;                                                      % For the next cycle, the potential to assign is greater
    %         pause(0.01)
        else
            done = true;                                                        % When the already_visited matrix is full of ones, stop the cycle
        end
    end

    % Plotting the grid-map 
%         figure()
%         imagesc(M); colormap parula; colorbar;                                  % Show the result as a colored figure

    %% Path planning as cells sequences
    
    % Finding q0 in the grid-map
    initial_indices = [0,0];
    
    for j= 0 : box_len/d - 1
        
        for i= 0 : box_len/d - 1
            
           center_vec_temp = fix([ A(1) + d/2 + d*i; (A(2)) + d/2 + d*j ] * 100) / 100;
           d_temp = sqrt((q_k(1) - center_vec_temp(1))^2 + (q_k(2) - center_vec_temp(2))^2);   % Distance between the goal and the center of a cell
           
           if d_temp <= d/2 * sqrt(2)                                                          % If the distance is lower than the distance between the center and the vertix of the cell, the point is inside it                   
               initial_indices = [length(M)-j,1+i];
               break
           end
           
        end
        
        if initial_indices(1)~=0 && initial_indices(2) ~=0
            break                                                                              % With the double break the two loop are stopped if the initial indices is found
        end
        
    end

    % Initialization of the vector with the path as cell sequence and as
    % center of cell sequence
    cell_path = zeros(M(initial_indices(1),initial_indices(2))+1,2);                           % Cells sequence
    cell_path(1,:) = initial_indices;               

    center_from_path = zeros(M(initial_indices(1),initial_indices(2))+1,2);                    % Sequence of the centers of the cells 
    center_from_path(1,:) = [A(1) + d/2 + d * (initial_indices(2)-1); A(2) + d/2 + d * (length(M)-initial_indices(1))];

     while ~isequal(cell_path(end,:),final_indices) 

        % Starting from the initial cell, explore its adjacent cells
        % choosing the first  with potential equal to the reference one -1.
        % Repeat until reach the goal
        
        for i=1:length(cell_path(:,1)) 
                adjacent_cells = [];
                if(cell_path(i,1)+1 <= box_len/d)
                    right = cell_path(i,:)+[1,0];
                    adjacent_cells = [adjacent_cells;right];
                end

                if(cell_path(i,1)-1 > 0)
                    left = cell_path(i,:)-[1,0];
                    adjacent_cells = [adjacent_cells;left];
                end            

                if(cell_path(i,2)+1 <= box_len/d)
                    up = cell_path(i,:)+[0,1];
                    adjacent_cells = [adjacent_cells;up];
                end

                if(cell_path(i,2)-1 > 0)
                    down = cell_path(i,:)-[0,1];
                    adjacent_cells = [adjacent_cells;down];
                end

                for k=1:length(adjacent_cells(:,1))
                    if M(adjacent_cells(k,1),adjacent_cells(k,2)) == M(cell_path(i,1),cell_path(i,2))-1

                       cell_path(i+1,:) = adjacent_cells(k,:);              % Building the path as a sequence of cell
                       
                       % Building the path as sequence of the center of the choosing cells
                       center_from_path(i+1,:) = [A(1) + d/2 + d*(cell_path(i+1,2)-1); A(2) + d/2 + d*(length(M)-cell_path(i+1,1)) ]; 
                       break                                                % Break to avoid to add more than one cell with the same potential                   
                    end
                end
        end

     end

%     points = cat(1,center_from_path,[qg(1),qg(2)]);                         % Adding the goal as the last point of the path
    points = center_from_path;
%     points(end-4:end,:) = [];
    points = cat(1,[q0(1),q0(2)],points);                                   % Adding the initial configuration as the first point of the path
    points_path = opt_cycle(points, s1, s2);

    %%
    % Artificial potential to obtain the path between each point from q0 to qg
    % considering the center of the cells belonging to the path already computed
     
    q = [q_k(1),q_k(2)];
    traj_planned=q;                                                         % The path starts from q0, only cartesian coordinates are considered
    ka = 0.500;
    qd = [0 0];
    
    for i=1:length(points)
        e = points(i,:) - q;
        while(norm(e) > 0.01)
            e = points(i,:) - q;
            
            % Attractive potential
            Ua1 = 1/2 * ka * norm(e)^2;
            fa1 = ka * e;

            Ua2 = ka * norm(e);
            fa2 = ka * e / norm(e);

            if(norm(e) <= 1)  
                fa = fa1;
                Ua = Ua1;
            else  
                 fa = fa2;
                 Ua = Ua2;
            end   

            % Total potential -- only attractive potential
            ft = fa;
            Ut = Ua;

             % Euler integration
             T = 0.001/norm(ft);             
             q = q + T*ft;
             qd = [qd; ft];
             traj_planned = cat(1,traj_planned,q);                          % Building the path 
        end  
    end
%      close all
end