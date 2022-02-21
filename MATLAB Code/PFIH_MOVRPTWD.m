function [rute0,infeasibility,E0]=PFIH_MOVRPTWD(xy,wt,wd,delta,kota,kendaraan,itmax)
a1=.7; %alpha
a2=.1; %beta
a3=.2; %gamma
d=squareform(pdist(xy,'euclidean')); %Jarak antar customer dalam mil
D=kota(:,1); %Demand tiap customer
a=kota(:,2); %Jam buka tiap customer
b=kota(:,3); %Jam tutup tiap customer
h=kota(:,4); %Ketinggian tiap customer
[ncit,~]=size(kota);
[nveh,~]=size(kendaraan);
rute_now=[ones(nveh,1,itmax) zeros(nveh,ncit-1,itmax) ones(nveh,1,itmax)];
it=1;
while it<=itmax
    kendaraan0=kendaraan(1,:);
    rute0=[1 zeros(1,ncit-1) 1];
    for i=2:ncit %biaya penyisipan
        cos_theta(i)=(xy(i,1)-xy(1,1))/d(1,i);
        c(i)=-a1*(d(1,i))+a2*b(i)+a3*(acosd(cos_theta(i))/360)*(d(1,i));
    end
    cos_theta=cos_theta(2:end);
    c=c(2:end);
    [~,idk]=sort(c);
    rute0(1,2:ncit)=idk+1;
    [rute0,~,~,~,infeasibility]=feasibilitas(xy,rute0,kendaraan0,kota);
    clear z
    z=2;
    while infeasibility>0
        rute0(z,:)=[1 zeros(1,ncit-1) 1];
        kendaraan1=kendaraan(1:z,:);
        qt=kendaraan1(:,1); %Kapasitas truk
        qd=kendaraan1(:,2); %Kapasitas drone
        vt=kendaraan1(:,3); %Kecepatan truk
        vd=kendaraan1(:,4); %Kecepatan drone
        r=kendaraan1(:,5); %Batas total waktu maksimum truk
        [~,~,~,~,infeasibility1]=feasibilitas(xy,rute0(z-1,:),kendaraan1(z-1,:),kota);
        while infeasibility1>0
            rute_inf=find(rute0(z-1,2:end-1)~=0)+1;
            rute_rot=randperm(length(rute_inf));
            rute0([z-1 z],rute_inf(rute_rot(1)))=rute0([z z-1],rute_inf(rute_rot(1)));
            [rute0(z-1,:),~,~,~,infeasibility1]=feasibilitas(xy,rute0(z-1,:),kendaraan1(z-1,:),kota);
            rute0(z-1,:)=norm_rute(rute0(z-1,:));
        end
        [rute0,~,~,~,infeasibility]=feasibilitas(xy,rute0,kendaraan1,kota);
        rute0=norm_rute(rute0);
        z=z+1;
    end
    [rute0,E0(it)]=biaya(xy,wt(1:z-1),wd(1:z-1),rute0,kendaraan(1:z-1,:),kota,delta(1:z-1));
    rute0=norm_rute(rute0);
    if it==1
        [nveh1,~]=size(rute0);
        rute_now(1:nveh1,:,it)=rute0;
    else
        if sum(sum(rute_now(:,:,it)')'~=2)<sum(sum(rute_now(:,:,it-1)')'~=2)
            [nveh1,~]=size(rute0);
            rute_now(1:nveh1,:,it)=rute0;
        else
            if E0(it)>E0(it-1)
                E0(it)=E0(it-1);
                rute_now(:,:,it)=rute_now(:,:,it-1);
            else
                [nveh1,~]=size(rute0);
                rute_now(1:nveh1,:,it)=rute0;
            end
        end
    end
    clear rute0 nveh1
    it=it+1;
end
rute0=rute_now(:,:,it-1);
rute0((sum(rute0')'==2),:)=[];
E0=E0(it-1);
end