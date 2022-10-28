function draw(I,rI,rQ)
figure;
root = sqrt(2)/2;
%”≥…‰æÿ’Û
MappingMat = [[1,0];[root,root];[-root,root];[0,1];[root,-root];[0,-1];[-1,0];[-root,-root]];
L=length(I)
for i=1:L
    % Draw points with different colors according to different values
    switch(I(i))
        case 1
            plot(rI(i),rQ(i),'*','color','r');hold on;
           
        case 2
            plot(rI(i),rQ(i),'*','color','g');hold on;
           
        case 3
            plot(rI(i),rQ(i),'*','color','b');hold on;
   
        case 4
            plot(rI(i),rQ(i),'*','color','c');hold on;
   
        case 5
            plot(rI(i),rQ(i),'*','color','m');hold on;
            
        case 6
            plot(rI(i),rQ(i),'*','color','y');hold on;
        
        case 7
            plot(rI(i),rQ(i),'*','color','k');hold on;
          
        case 8
            plot(rI(i),rQ(i),'*','color','[0.5,0.5,0.5]');hold on;
            
    end
end
x = -4:0.1:4;
y = -2:0.1:2;
% plot(x,zeros(length(x)),'k');hold on;
% plot(zeros(length(y)),y,'k');hold on;
axis equal
axis([-2,2,-2,2]);
end