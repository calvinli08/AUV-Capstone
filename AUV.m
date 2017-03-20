classdef AUV < handle
    %Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        position_x % Row
        position_y % Column
        previous_x %for graphing 
        previous_y %for graphing 
        velocity  %Step size for traversal
        border_x  % World Boundary - used for sparse traversal.  
        border_y  % World Boundary - used for sparse traversal
        current_knowledge % Knowledge of the world
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
  
        end
        
        %Traverses by velocity amount
        function traverse(thisAUV, direction)  
            %N,S traverses rows, E,W traverses Columns.  We are assuming
            %1,1 is the top left corner of the array
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
        
        %Flip direction for lawnmower sweep
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
        
        %Grabs a sample
        function sample(thisAUV, world)
            if(thisAUV.position_x <= thisAUV.border_x && thisAUV.position_y <= thisAUV.border_y)
                thisAUV.current_knowledge(thisAUV.position_x,thisAUV.position_y) = world(thisAUV.position_x,thisAUV.position_y);
            end
        end
        
        %Sparse Traversal
        function sparseTraverse(thisAUV, step_size, world, direction_long, direction_short)
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
                direction_long = thisAUV.switchDirection(direction_long);
                %Short
                for x = 1:step_size
                    thisAUV.sample(world)
                    thisAUV.traverse(direction_short)
                end
            end
            
           for i = 1 : long_lim
                thisAUV.sample(world)
                thisAUV.traverse(direction_long);
           end
            
        end
        
        function gradient(thisAUV)
            
        end
        
        

    end
    
end

