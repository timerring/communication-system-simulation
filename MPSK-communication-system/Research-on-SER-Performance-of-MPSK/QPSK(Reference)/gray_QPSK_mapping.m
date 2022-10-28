function[gQm]=gray_QPSK_mapping(a,b,length)
% Generate a 2¡Álength matrix
gQm=zeros(2,length);
% map by gray code
for i=1:length
    if(a(i)==0&&b(i)==0)
        gQm(1,i)=1;gQm(2,i)=0; 
    elseif(a(i)==0&&b(i)==1)
        gQm(1,i)=0;gQm(2,i)=1;
    elseif(a(i)==1&&b(i)==1)
        gQm(1,i)=-1;gQm(2,i)=0;
    elseif(a(i)==1&&b(i)==0)
        gQm(1,i)=0;gQm(2,i)=-1;
    end
end
end