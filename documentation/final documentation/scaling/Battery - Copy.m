function SOCnn = Battery( I , SOCn , dt)

%dt = 1; %sec
R = 0.01; 
Nom_i = 5.2; %A
Nom_c = 5.2; %Ahr
k = 1;

%%%%cp,DODn,CRn,CRnn;s
%Calc. Peukert capacity from nominal
cp=(Nom_i^k)*Nom_c/Nom_i;
DODn = 1 - SOCn/100;
CRn = DODn*cp;

%I = (OCV - sqrt((OCV*OCV) - (4*R*P_batt)))/(2*R);
CRnn = CRn +(dt*I^k / 3600);

SOCnn = 100-(CRnn/cp)*100;

if SOCnn < 0 
    SOCnn = 0;
end

end

