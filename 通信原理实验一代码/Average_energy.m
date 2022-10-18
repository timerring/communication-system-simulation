
function Eb=Average_energy(xn,L)
Eb=0;
for i = 1 : L
    Eb=Eb+abs(xn(i))^2;
end
    Eb=Eb/L;
end
