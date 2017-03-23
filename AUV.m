classdef AUV < handle
    % AUV Class

    
    properties
        %Self-Knowledge
        position_x % Row
        position_y % Column
        velocity  %Step size for traversal
        
        %Arrays that track where the AUV has been
        previous_x %for graphing 
        previous_y %for graphing 
        
        %World Knowledge
        border_x  % World Boundary - used for sparse traversal.  
        border_y  % World Boundary - used for sparse traversal
        current_knowledge % Raw Knowledge of the world
        points_of_interest % Either just the coordinates or a full array.  I'm using a full array right now.
    end
    
    methods
        %Constructor
        function obj = AUV(x,y,v,bound_x,bound_y)
                if nargin == 0
                    %Default args
                    x = 1;
                    y = 1;
                    v = 1;
                    bound_x = 100;
                    bound_y = 100;
                end
                
                obj.position_x = x;
                obj.position_y = y;
                obj.previous_x = x;
                obj.previous_y = y;
                obj.velocity = v;
                obj.border_x = bound_x;
                obj.border_y = bound_y;
                obj.current_knowledge = zeros(bound_x,bound_y);
                obj.points_of_interest = zeros(bound_x,bound_y);
        end
        
        %Traverses by velocity amount
        %Direction is a string: 'N'(North), 'S'(South), 'E' (East),'W'(West)
        %N,S traverses rows, E,W traverses Columns.  
        %We are assuming (1,1) is the top left corner of the array
        function traverse(thisAUV, direction)  
            switch direction
                case 'S'
                    if(thisAUV.position_x + thisAUV.velocity <= thisAUV.border_x)
                        thisAUV.position_x = thisAUV.position_x + thisAUV.velocity;
                    else
                        warning('Reached edge')
                    end
                case 'N'
                    if(thisAUV.position_x - thisAUV.velocity > 0)
                        thisAUV.position_x = thisAUV.position_x - thisAUV.velocity;
                    else
                       warning('Reached edge')
                    end
                case 'E'
                    if(thisAUV.position_y + thisAUV.velocity <= thisAUV.border_y)
                        thisAUV.position_y = thisAUV.position_y + thisAUV.velocity;
                    else
                       warning('Reached edge')
                    end
                case 'W'
                    if(thisAUV.position_y + thisAUV.velocity > 0)
                        thisAUV.position_y = thisAUV.position_y - thisAUV.velocity;
                    else
                       warning('Reached edge')
                    end
                otherwise
                    warning('Direction must be a string: N,S,W,E')
            end
            thisAUV.previous_x = [thisAUV.previous_x, thisAUV.position_x];
            thisAUV.previous_y = [thisAUV.previous_y, thisAUV.position_y];
            
        end
        
        % Function takes in a direction and flips it.
        function direction = switchDirection(thisAUV,direction)
            switch direction
                case 'S'
                    direction = 'N';
                case 'N'
                    direction = 'S';
                case 'E'
                    direction = 'W';
                case 'W'
                    direction = 'E';
            end
        
        end
        
                % Function takes in a direction and flips it.
        function direction = rotatecw(direction)
            switch direction
                case 'N'
                    direction = 'E';
                case 'E'
                    direction = 'S';
                case 'S'
                    direction = 'W';
                case 'W'
                    direction = 'N';
            end
        
        end
        
        
        
        %Takes the value at the current position in the world and fills it
        %into current_knowledge.
        %I would like it to also fill in points of interest
        function sample(thisAUV, world)
            if(thisAUV.position_x <= thisAUV.border_x && thisAUV.position_y <= thisAUV.border_y)
                thisAUV.current_knowledge(thisAUV.position_x,thisAUV.position_y) = world(thisAUV.position_x,thisAUV.position_y);
            end
        end
        %Square Traversal
        %step_size is the additional amount it traverses down the longest
        %path in the square. In other words, the 'space' between squares.
        %world - 2d Array of the world
        %target point - this is the interest point from which we wish to
        %begin our traversal.
        function squareTraverse( thisAUV, step_size, end_point, threshold, world, targetPointx, targetPointy)
            ptp(thisAUV, targetPointx, targetPointy);
            direction = 'N';
            for n = 1 : end_point
                for m = 1:2
                    for i = 1 : n
                    thisAUV.sample(world)
                    thisAUV.traverse(direction)
                    end
                    direction = rotatecw(direction);
                end
            end
        end
        
        %Point to point traversal
        %thisAUV is the auv in question, with its current location.
        function ptp( thisAUV, targetPointx, targetPointy)
            x = targetPointx - thisAUV.position_x;
            y = targetPointy - thisAUV.position_y;
            absx = Abs(x);
            absy = Abs(y);
            directionh = 'E';
            directionv = 'N';
            if (targetPointx < thisAUV.position_x)
                directionh = 'W';
            else 
                directionh = 'E';
            end
            if (targetPointy < thisAUV.position_y)
                directionv = 'S';
            else 
                directionv = 'N';
            end

            while( absx > 0 && absy > 0 )
                %Travel in a diagonal towards the goal
                absx--;
                absy--;
                thisAUV.traverse(directionh);
                thisAUV.traverse(directionv);
            end
            while(absx > 0)
                thisAUV.traverse(directionh);
                absx--;
            end
            while(absy > 0)
                thisAUV.traverse(directionv);
                absy--;
            end
        end
        
        
        %Sparse Traversal
        %step_size is the amount it traverses down the short length of the
        % lawnmower traversal.
        %threshold - Value that represents a critical amount of pollution;
        % we should start a dense traversal if we see this
        %World - 2d Array of the world
        %direction_long - the long direction of our lawnmower traversal
        %direction_short - the short side of our lawnmower traversal
        function sparseTraverse(thisAUV, step_size,threshold, world, direction_long, direction_short)
            %Goes down the full length of direction_long
            %Traverses direction_short for step_size duration
            
            switch direction_long
                case {'E','W'}
                    long_lim = thisAUV.border_y;
                    short_lim = thisAUV.border_x;
                otherwise
                    long_lim = thisAUV.border_x;
                    short_lim = thisAUV.border_y;
            end
            
            
            %Need a condition to stop sparseTraverse if high pollution is
            %detected
            for j = 1:step_size:short_lim
                %Long
                for i = 1 : long_lim - 1
                    thisAUV.sample(world)
                    thisAUV.traverse(direction_long);
                end
                %Go the other way
                direction_long = thisAUV.switchDirection(direction_long);
                %Short
                for x = 1:step_size
                    thisAUV.sample(world)
                    thisAUV.traverse(direction_short)
                end
            end
            
           % The last leg of the traversal.  Comment this out and see what
           % happens on the figure; we lose the last row of data.
           % If you have a better solution please feel free this fix this
           for i = 1 : long_lim
                thisAUV.sample(world)
                thisAUV.traverse(direction_long);
           end
            
        end
        
        %Rates the pollution level
        function level = pollution_level(thisAUV,pollution_value)
            if(pollution_value <= 0.3)
                level = 0;
            else
                level = 1;
            %elseif(pollution_value <= 0.44)
            %    level = 1;
            %elseif (pollution_value <= 0.58)
            %    level = 2;
            %elseif (pollution_value <= 0.72)
            %    level = 3;
            %elseif (pollution_value <= 0.86)
            %    level = 4;
            %elseif (pollution_value <= 1)
            %    level = 5;
            
            end
        end
        
        

    end
    
end

