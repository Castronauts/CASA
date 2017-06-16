%%%
% RandomNoRepeat.m
% This function will continue to call randi() until it generates a random
% vector with no repetition of values
% Inputs: Min value, Max value, array size (must be equal to or less than
% max difference between max value and min value)
% Outputs: array
%%%
function outVec= RandomNoRepeat(minV,maxV,aSize)
    check= 0;
    outVec= randi([minV maxV],1,aSize);
    while check == 0
        check= 1;
        for i= 1:aSize
            for j= 1:aSize
                if (i~=j && outVec(i)==outVec(j))
                    check= 0;
                    outVec= randi([minV maxV],1,aSize);
                    break;
                    break;
                end
            end
        end
    end
end