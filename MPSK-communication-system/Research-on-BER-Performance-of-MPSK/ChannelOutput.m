% Superimpose the horizontal and vertical coordinates of the mapped signal respectively to output the two orthogonal noises
function[y]=ChannelOutput(y1,n)
y=zeros(2,length(y1));
y(1,:)=y1(1,:)+n(1,:);
y(2,:)=y1(2,:)+n(2,:);
end
