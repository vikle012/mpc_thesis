function [Psi] = Psi_thr(pus,pds,param)
Pi = pds./pus;
   c1 = -0.036040668514975;
   c2 = 0.756040668514975;
   c3 = 0.999500000000000;
   c4 = 4.548945000000000;
Psi = c1.*sqrt( 1 - (Pi.*c2).^c3 ) + c4;

end