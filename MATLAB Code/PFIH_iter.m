function [rute0,biaya]=PFIH_iter(xy,wt,wd,delta,kota,kendaraan,it_PFIH)
[rute1,~,biaya1]=PFIH_MOVRPTWD(xy,wt,wd,delta,kota,kendaraan,it_PFIH);
[rute2,~,biaya2]=PFIH_MOVRPTWD(xy,wt,wd,delta,kota,kendaraan,it_PFIH);
[rute3,~,biaya3]=PFIH_MOVRPTWD(xy,wt,wd,delta,kota,kendaraan,it_PFIH);
[rute4,~,biaya4]=PFIH_MOVRPTWD(xy,wt,wd,delta,kota,kendaraan,it_PFIH);
[rute5,~,biaya5]=PFIH_MOVRPTWD(xy,wt,wd,delta,kota,kendaraan,it_PFIH);
[rute6,~,biaya6]=PFIH_MOVRPTWD(xy,wt,wd,delta,kota,kendaraan,it_PFIH);
[rute7,~,biaya7]=PFIH_MOVRPTWD(xy,wt,wd,delta,kota,kendaraan,it_PFIH);
[rute8,~,biaya8]=PFIH_MOVRPTWD(xy,wt,wd,delta,kota,kendaraan,it_PFIH);
[rute9,~,biaya9]=PFIH_MOVRPTWD(xy,wt,wd,delta,kota,kendaraan,it_PFIH);
[rute10,~,biaya10]=PFIH_MOVRPTWD(xy,wt,wd,delta,kota,kendaraan,it_PFIH);
[nveh1,~]=size(rute1);
[nveh2,~]=size(rute2);
[nveh3,~]=size(rute3);
[nveh4,~]=size(rute4);
[nveh5,~]=size(rute5);
[nveh6,~]=size(rute6);
[nveh7,~]=size(rute7);
[nveh8,~]=size(rute8);
[nveh9,~]=size(rute9);
[nveh10,~]=size(rute10);
nvehx=[nveh1 nveh2 nveh3 nveh4 nveh5 nveh6 nveh7 nveh8 nveh9 nveh10];
[~,idknveh]=min(nvehx);
if idknveh==1
    rute0=rute1;
    biaya=biaya1;
elseif idknveh==2
    rute0=rute2;
    biaya=biaya2;
elseif idknveh==3
    rute0=rute3;
    biaya=biaya3;
elseif idknveh==4
    rute0=rute4;
    biaya=biaya4;
elseif idknveh==5
    rute0=rute5;
    biaya=biaya5;
elseif idknveh==6
    rute0=rute6;
    biaya=biaya6;
elseif idknveh==7
    rute0=rute7;
    biaya=biaya7;
elseif idknveh==8
    rute0=rute8;
    biaya=biaya8;
elseif idknveh==9
    rute0=rute9;
    biaya=biaya9;
else
    rute0=rute10;
    biaya=biaya10;
end
end