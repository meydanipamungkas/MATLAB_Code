function [rute,E_SA]=SA_MOVRPTWD(xy,wt,wd,delta,kota,kendaraan,c,itmax)
d=squareform(pdist(xy,'euclidean')); %Jarak antar customer dalam mil
D=kota(:,1); %Demand tiap customer
a=kota(:,2); %Jam buka tiap customer
b=kota(:,3); %Jam tutup tiap customer
h=kota(:,4); %Ketinggian tiap customer
qt=kendaraan(:,1); %Kapasitas truk
qd=kendaraan(:,2); %Kapasitas drone
vt=kendaraan(:,3); %Kecepatan truk
vd=kendaraan(:,4); %Kecepatan drone
r=kendaraan(:,5); %Batas total waktu maksimum truk
it_PFIH=1;
it=1;
itcon=10;
n=1;
nmax=10;
E_SA=zeros(itmax,1);
[rute0,~,E_SA(1,1)]=PFIH_MOVRPTWD(xy,wt,wd,delta,kota,kendaraan,it_PFIH);
[ncit,~]=size(kota);
[nveh,~]=size(rute0);
rute=zeros(nveh,ncit+1,itmax);
deltaE=10;
rute(:,:,1)=rute0;
T0=E_SA(1,1);
T=T0;
while it<=itmax
    if deltaE>1e-8
        while n<=nmax
            rute1=zeros(nveh,ncit+1);
            rute2=rute1;
            if it==1
                rute(:,:,it)=rute0;
            else
                rute(:,:,it)=rute(:,:,it-1);
            end
            r1=rand; %flip/swap/slide customer
            rute1=rute(:,:,it);
            [r,c]=size(rute1);
            rute1=reshape((rute1(:,2:end))',1,r*(c-1));
            rute1=norm_rute2(rute1);
            rute1(rute1==0)=[];
            rute1(end)=[];
            r11=sort(ceil((ncit-1)*rand(1,2)));
            I=r11(1);
            J=r11(2);
            if r1<=.33
                rute1(I:J)=fliplr(rute1(I:J)); %flip customer
            elseif r1<=.67 && r1>.33
                rute1([I J])=rute1([J I]); %swap customer
            else
                rute1([I:J])=rute1([(I+1):J I]); %slide customer
            end
            rute1=norm_rute3(rute1,kendaraan(1:nveh,:),kota);
            [rute2,~,~,~,infeasibility]=feasibilitas(xy,rute1,kendaraan(1:nveh,:),kota);
            if infeasibility==0
                rute2=norm_rute(rute2);
                [~,E_SA1(it,1)]=biaya(xy,wt(1:nveh,:),wd(1:nveh,:),rute2,kendaraan(1:nveh,:),kota,delta(1:nveh,:));
                if E_SA1(it,1)<E_SA(it,1)
                    rute(:,:,it)=rute2;
                    E_SA(it,1)=E_SA1(it,1);
                else
                    r=rand;
                    if r<=exp((E_SA(it,1)-E_SA1(it,1))/T) %kriteria metropolis
                        rute(:,:,it)=rute2;
                        E_SA(it,1)=E_SA1(it,1);
                    end
                end
            end
            n=n+1;
        end
    else
        break
    end
    if it<itcon
        deltaE=10;
    else
        deltaE=std(E_SA((it-(itcon-1)):it,1));
    end
    T=c*T;
    n=1;
    it=it+1;
end
for j=1:it-1
    [rute1(:,:,j),E_SA(j,1)]=biaya(xy,wt(1:nveh,:),wd(1:nveh,:),rute(:,:,j),kendaraan(1:nveh,:),kota,delta(1:nveh,:));
end
E_SA(E_SA==0)=[];
[E_SA,idk]=min(E_SA(:,1));
rute=rute1(:,:,idk);
rute=norm_rute(rute);
rute=norm_rute2(rute);
rute(sum(rute')==2,:)=[];
end