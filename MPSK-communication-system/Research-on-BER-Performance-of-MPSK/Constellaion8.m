function Constellaion8(y1,y2)
% y1 is the octal symbol sequence, y2 is the signal output by the noisy channel
figure 
for i=1:length(y1)     
    switch y1(i)      
        % Constellation point of %0 signal
        case {0}
            plot(y2(1,i),y2(2,i),'B.');hold on;grid on;
            axis([-2 2 -2 2]);
            line([2,-2],[0,0],'linewidth',1,'color','black');
            line([0,0],[2,-2],'linewidth',1,'color','black');
        case {1}
            plot(y2(1,i),y2(2,i),'Y.');hold on;grid on;
            axis([-2 2 -2 2]);
            line([2,-2],[0,0],'linewidth',1,'color','black');
            line([0,0],[2,-2],'linewidth',1,'color','black');
        case {2}
            plot(y2(1,i),y2(2,i),'R.');hold on;grid on;
            axis([-2 2 -2 2]);
            line([2,-2],[0,0],'linewidth',1,'color','black');
            line([0,0],[2,-2],'linewidth',1,'color','black');
        case {3}
            plot(y2(1,i),y2(2,i),'G.');hold on;grid on;
            axis([-2 2 -2 2]);
            line([2,-2],[0,0],'linewidth',1,'color','black');
            line([0,0],[2,-2],'linewidth',1,'color','black');
            
    end  
end
hold off;
end
