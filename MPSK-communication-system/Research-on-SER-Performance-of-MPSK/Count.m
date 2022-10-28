function [errbit,errsymbol]=Count(result,I,sourceSeq,sourceSym)
% result: the sequence after the decision
% I: symbol after decision
% sourceSeq: original bit sequence
% sourceSym: original symbol sequence
errbit=0;
errsymbol=0;
for i=1:length(sourceSeq)
    if(sourceSeq(i)~=result(i))
        errbit=errbit+1;
    end
end
for i=1:length(I)
    if(I(i)~=sourceSym(i))
        errsymbol=errsymbol+1;
    end
end
end
