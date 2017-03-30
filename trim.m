%poi 
function trim = trim(poi, thresh)
%poi = [1 1; 2 2; 3 3; 4 4; 10 12; 13 14; 22 1; 1 30; 54 54; 600 1];

%thresh = 2;
    a = size(poi);
    a = a(1); %a is now the number of rows there are in poi.
    i = 1;
    while i < a
        j = i+1;
        while j < a
            if(pdist([poi(i,:);poi(j,:)]) < thresh)
                if( i > 1 ) 
                   % poi(i-1,:) = (poi(i,:) + poi(i-1,:))/2;
                    poi = [poi(1:i-1, :); poi(i+1 : a, :)];
                else
                    %poi(2,:) = (poi(2,:) + poi(1,:))/2;
                    poi = poi(2:a,:);
                end
                a = a-1;
            else
                j = j+1;
            end
        end
        i = i+1;
    end
    trim = poi;
end