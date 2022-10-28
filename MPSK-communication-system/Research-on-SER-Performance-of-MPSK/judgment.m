% Judgment Criterion: Minimum Euclidean Distance Criterion
function [result,I]=judgment(rI,rQ)
root=sqrt(2)/2;
L = length(rI);
I=zeros(L,1);
mapl = [[1,0];[root,root];[-root,root];[0,1];[root,-root];[0,-1];[-1,0];[-root,-root]];
for i=1:L
    index=0;
    minp=100;
    for j=1:8
        % Traverse each coordinate
        % find the point with the smallest Euclidean distance from the point, and record it.
        if((mapl(j,1)-rI(i))^2+(mapl(j,2)-rQ(i))^2<minp)
            minp=(mapl(j,1)-rI(i))^2+(mapl(j,2)-rQ(i))^2;
            index=j;
        end
    end
    I(i)=index;
end

% According to the judgment point, make the corresponding assignment
result = zeros(3*L,1);
for i=1:L
    switch(I(i))
        case 1
            [result(3*i-2),result(3*i-1),result(3*i)]=SetValue(0,0,0);
        case 2
            [result(3*i-2),result(3*i-1),result(3*i)]=SetValue(0,0,1);
        case 3
            [result(3*i-2),result(3*i-1),result(3*i)]=SetValue(0,1,0);
        case 4
            [result(3*i-2),result(3*i-1),result(3*i)]=SetValue(0,1,1);
        case 5
            [result(3*i-2),result(3*i-1),result(3*i)]=SetValue(1,0,0);
        case 6
            [result(3*i-2),result(3*i-1),result(3*i)]=SetValue(1,0,1);
        case 7
            [result(3*i-2),result(3*i-1),result(3*i)]=SetValue(1,1,0);
        case 8
            [result(3*i-2),result(3*i-1),result(3*i)]=SetValue(1,1,1);
    end
end

end
