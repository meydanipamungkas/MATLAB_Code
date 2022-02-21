function [rute,E_ACO]=ACO_MOVRPTWD(xy,wt,wd,delta,kota,kendaraan,N,itmax)
d=squareform(pdist(xy,'euclidean')); %Jarak antar customer dalam mil
D=kota(:,1); %Demand tiap customer
a=kota(:,2); %Jam buka tiap customer
b=kota(:,3); %Jam tutup tiap customer
height=kota(:,4); %Ketinggian tiap customer
qt=kendaraan(:,1); %Kapasitas truk
qd=kendaraan(:,2); %Kapasitas drone
vt=kendaraan(:,3); %Kecepatan truk
vd=kendaraan(:,4); %Kecepatan drone
r=kendaraan(:,5); %Batas total waktu maksimum truk
it=1;
[ncit,~]=size(kota);
[nveh,~]=size(kendaraan);
rute=zeros(nveh,ncit+1,N,itmax);
rute(:,[1 end],:,:)=1;
iter=ncit-2;
alpha=1;
beta=3;
rho=.5;
tau=.01*ones(ncit,ncit,nveh);
E_ACO=zeros(N,itmax);
while it<=itmax
    for i=1:ncit
        for j=1:ncit
            for k=1:nveh
                for l=1:N
                    if d(i,j)==0
                        h(i,j,k,l)=0; %visibilitas
                    else
                        h(i,j,k,l)=1/d(i,j);
                    end
                end
            end
        end
    end
    h(:,1,:,:)=0;
    depot=zeros(N,ncit);
    for l=1:N %probabilitas transisi dari kota pertama
        depot(l,:)=((tau(1,:,1).^alpha).*(h(1,:,1,l).^beta))./sum(((tau(1,:,1).^alpha).*(h(1,:,1,l).^beta))');
    end
    depot=cumsum(depot')';
    r=rand(N,1);
    city1=zeros(N,1);
    for l=1:N
        for i=1:ncit-1
            if r(l)<=depot(l,i+1)
                city1(l,1)=i+1;
                break
            end
        end
    end
    rute1=[ones(N,1) zeros(N,ncit-1) ones(N,1)];
    rute1(:,2)=[city1];
    infeasibility1=zeros(N,1);
    for l=1:N
        [~,~,~,~,infeasibility1(l,1),~,~,~,~]=feasibilitas(xy,rute1(l,:),kendaraan(1,:),kota);
    end
    while sum(infeasibility1)>0
        idk1=find(infeasibility1~=0);
        r(idk1,1)=rand(sum(infeasibility1~=0),1);
        for l=[idk1']
            for i=1:ncit-1
                if r(l)<=depot(l,i+1)
                    city1(l,1)=i+1;
                    break
                end
            end
        end
        rute1(:,2)=[city1];
        for l=1:N
            [~,~,~,~,infeasibility1(l,1),~,~,~,~]=feasibilitas(xy,rute1(l,:),kendaraan(1,:),kota);
        end
    end
    for l=1:N
        rute(1,:,l,it)=rute1(l,:);
    end
    for l=1:N
        h(:,rute1(l,2),1,l)=0;
    end
    nveh0=ones(N,1);
    while iter>0
        rw=zeros(N,ncit);
        for l=1:N %probabilitas transisi
            if sum(sum(h(:,:,nveh0(l,1),l)')')==0
                nveh0(l,1)=nveh0(l,1)+1;
                rw(l,:)=((tau(1,:,nveh0(l,1)).^alpha).*(h(1,:,nveh0(l,1),l).^beta))./sum(((tau(1,:,nveh0(l,1)).^alpha).*(h(1,:,nveh0(l,1),l).^beta))');
            else
                if rute(nveh0(l,1),ncit-iter,l,it)==0
                    rw(l,:)=((tau(1,:,nveh0(l,1)).^alpha).*(h(1,:,nveh0(l,1),l).^beta))./sum(((tau(1,:,nveh0(l,1)).^alpha).*(h(1,:,nveh0(l,1),l).^beta))');
                else
                    rw(l,:)=((tau(rute(nveh0(l,1),ncit-iter,l,it),:,nveh0(l,1)).^alpha).*(h(rute(nveh0(l,1),ncit-iter,l,it),:,nveh0(l,1),l).^beta))./sum(((tau(rute(nveh0(l,1),ncit-iter,l,it),:,nveh0(l,1)).^alpha).*(h(rute(nveh0(l,1),ncit-iter,l,it),:,nveh0(l,1),l).^beta))');
                end
            end
        end
        rw=cumsum(rw')';
        r=rand(N,1);
        city=zeros(N,1);
        for l=1:N
            for i=1:ncit-1
                if r(l)<=rw(l,i+1)
                    city(l,1)=i+1;
                    break
                end
            end
        end
        for l=1:N
            rute(nveh0(l,1),ncit-iter+1,l,it)=city(l,1);
        end
        infeasibility=zeros(N,1);
        for l=1:N
            [~,~,~,~,infeasibility(l,1),~,~,~,~]=feasibilitas(xy,rute(:,:,l,it),kendaraan,kota);
        end
        while sum(infeasibility)>0
            rw=zeros(N,ncit);
            for l=1:N %probabilitas transisi
                if sum(sum(h(:,:,nveh0(l,1),l)')')==0
                    nveh0(l,1)=nveh0(l,1)+1;
                    rw(l,:)=((tau(1,:,nveh0(l,1)).^alpha).*(h(1,:,nveh0(l,1),l).^beta))./sum(((tau(1,:,nveh0(l,1)).^alpha).*(h(1,:,nveh0(l,1),l).^beta))');
                else
                    if rute(nveh0(l,1),ncit-iter,l,it)==0
                        rw(l,:)=((tau(1,:,nveh0(l,1)).^alpha).*(h(1,:,nveh0(l,1),l).^beta))./sum(((tau(1,:,nveh0(l,1)).^alpha).*(h(1,:,nveh0(l,1),l).^beta))');
                    else
                        rw(l,:)=((tau(rute(nveh0(l,1),ncit-iter,l,it),:,nveh0(l,1)).^alpha).*(h(rute(nveh0(l,1),ncit-iter,l,it),:,nveh0(l,1),l).^beta))./sum(((tau(rute(nveh0(l,1),ncit-iter,l,it),:,nveh0(l,1)).^alpha).*(h(rute(nveh0(l,1),ncit-iter,l,it),:,nveh0(l,1),l).^beta))');
                    end
                end
            end
            rw=cumsum(rw')';
            idk=find(infeasibility~=0);
            r(idk,1)=rand(sum(infeasibility~=0),1);
            for l=1:length(idk)
                for i=1:ncit-1
                    if r(idk(l))<=rw(idk(l),i+1)
                        city(idk(l),1)=i+1;
                        break
                    end
                end
            end
            for l=1:N
                rute(nveh0(l,1),ncit-iter+1,l,it)=city(l,1);
            end
            for l=1:N
                [~,~,~,~,infeasibility(l,1),~,~,~,~]=feasibilitas(xy,rute(:,:,l,it),kendaraan,kota);
            end
            if sum(infeasibility)>0
                idk=find(infeasibility~=0);
                clear infeas_veh infeas_veh1 rute2 rb_kosong rute_belum 
                rute2=zeros(N,ncit+1);
                rb_kosong=zeros(length(idk),1);
                for l=1:length(idk)
                    rb_kosong(l,1)=length([find(sum(h(:,:,nveh0(l,1),idk(l)))~=0)]);
                end
                rute_belum=zeros(length(idk),max(rb_kosong));
                for l=1:N
                    rute2(l,:)=rute(nveh0(l,1),:,l,it);
                end
                for l=1:length(idk)
                    rute_belum(l,1:length([find(sum(h(:,:,nveh0(l,1),idk(l)))~=0)]))=[find(sum(h(:,:,nveh0(l,1),idk(l)))~=0)];
                end
                [~,col_rb]=size(rute_belum);
                if col_rb>0
                    if col_rb==1
                        idk2=find(rute_belum==0);
                        sum_rb=rute_belum==0;
                    else
                        idk2=find(sum(rute_belum')'==0);
                        sum_rb=sum(rute_belum')'==0;
                    end
                    if sum(sum_rb)>0
                        for l=1:length(idk2)
                            h(:,:,nveh0(idk(idk2(l)),1),[idk(idk2(l))])=0; %masalah utama
                            rute([nveh0(idk(idk2(l)),1) nveh0(idk(idk2(l)),1)+1],ncit-iter+1,idk(idk2(l)),it)=rute([nveh0(idk(idk2(l)),1)+1 nveh0(idk(idk2(l)),1)],ncit-iter+1,idk(idk2(l)),it);
                            nveh0(idk(idk2(l)),1)=nveh0(idk(idk2(l)),1)+1;
                        end
                        rute_belum([idk2],:)=[];
                        idk([idk2],:)=[];
                    end
                    clear r_zero c_zero
                    [r_zero,c_zero]=find(rute_belum==0);
                    for i=1:length(r_zero)
                        rute_belum(r_zero(i),c_zero(i))=rute_belum(r_zero(i),c_zero(i)-1);
                        while rute_belum(r_zero(i),c_zero(i))==0
                            rute_belum(r_zero(i),c_zero(i))=rute_belum(r_zero(i),c_zero(i)-1);
                        end
                    end
                    for l=1:length(idk)
                        for i=1:col_rb
                            if rute_belum(l,i)~=0
                                rute2(idk(l),ncit-iter+1)=rute_belum(l,i);
                                [~,~,~,~,infeas_veh(i),~,~,~,~]=feasibilitas(xy,rute2(idk(l),:),kendaraan(nveh0(l,1),:),kota);
                            end
                        end
                    end
                end
                exist infeas_veh;
                if ans==1
                    infeas_veh1=infeas_veh==0;
                    if sum(infeas_veh1)==0
                        for l=1:length(idk)
                            h(:,:,nveh0(idk(l),1),[idk(l)])=0;
                            rute([nveh0(idk(l),1) nveh0(idk(l),1)+1],ncit-iter+1,idk(l),it)=rute([nveh0(idk(l),1)+1 nveh0(idk(l),1)],ncit-iter+1,idk(l),it);
                            nveh0(idk(l),1)=nveh0(idk(l),1)+1;
                            for k=1:N
                                [~,~,~,~,infeasibility(k,1),~,~,~,~]=feasibilitas(xy,rute(:,:,k,it),kendaraan,kota);
                            end
                        end
                        for i=1:N
                            if infeasibility(i,1)==0
                                h(:,rute(nveh0(i,1),ncit-iter+1,i,it),:,i)=0;
                            else
                                if iter==1
                                    h(:,rute(nveh0(i,1),ncit-iter+1,i,it),:,i)=0;
                                end
                            end
                        end
                    else
                        for l=1:length(idk)
                            h(:,:,nveh0(idk(l),1),[idk(l)])=0;
                            rute([nveh0(idk(l),1) nveh0(idk(l),1)+1],ncit-iter+1,idk(l),it)=rute([nveh0(idk(l),1)+1 nveh0(idk(l),1)],ncit-iter+1,idk(l),it);
                            nveh0(idk(l),1)=nveh0(idk(l),1)+1;
                            for k=1:N
                                [~,~,~,~,infeasibility(k,1),~,~,~,~]=feasibilitas(xy,rute(:,:,k,it),kendaraan,kota);
                            end
                        end
                    end
                else
                    for l=1:length(idk)
                        h(:,:,nveh0(idk(l),1),[idk(l)])=0;
                        rute([nveh0(idk(l),1) nveh0(idk(l),1)+1],ncit-iter+1,idk(l),it)=rute([nveh0(idk(l),1)+1 nveh0(idk(l),1)],ncit-iter+1,idk(l),it);
                        nveh0(idk(l),1)=nveh0(idk(l),1)+1;
                        for k=1:N
                            [~,~,~,~,infeasibility(k,1),~,~,~,~]=feasibilitas(xy,rute(:,:,k,it),kendaraan,kota);
                        end
                    end
                end
            end
        end
        for i=1:ncit
            for j=1:nveh
                for k=1:N
                    if rute(j,i,k,it)~=0
                        h(:,rute(j,i,k,it),:,k)=0;
                    end
                end
            end
        end
        iter=iter-1;
    end
    for i=1:N
        [~,E_ACO(i,it)]=biaya(xy,wt(1:nveh,:),wd(1:nveh,:),rute(:,:,i,it),kendaraan(1:nveh,:),kota,delta(1:nveh,:));
    end
    for i=1:ncit
        for j=1:ncit
            for k=1:nveh
                tau(i,j,k)=tau(i,j,k).*(1-rho);
            end
        end
    end
    delta_tau=zeros(ncit,ncit,nveh);
    for l=1:N
        rute1=norm_rute2(rute(:,:,l,it));
        [rr1,cr1]=size(rute1);
        for x=1:rr1
            for y=1:cr1-1
                if rute1(x,y)==0
                    continue
                elseif rute1(x,y+1)==0
                    delta_tau(rute1(x,y),1,x)=delta_tau(rute1(x,y),1,x)+1./E_ACO(l,it);
                else
                    delta_tau(rute1(x,y),rute1(x,y+1),x)=delta_tau(rute1(x,y),rute1(x,y+1),x)+1./E_ACO(l,it);
                end
            end
        end
        clear rute1 rr1 cr1
    end
    for i=1:ncit
        for j=1:ncit
            for k=1:nveh
                if i==j
                    delta_tau(i,j,k)=0;
                end
            end
        end
    end
    tau=tau+delta_tau; %update pheromone
    iter=ncit-2;
    it=it+1;
end
rute4=norm_rute4(tau,xy,kendaraan(1:nveh,:),kota);
[~,E_ACO4]=biaya(xy,wt(1:nveh,:),wd(1:nveh,:),rute4,kendaraan(1:nveh,:),kota,delta(1:nveh,:));
if E_ACO4<min(min(E_ACO))
    E_ACO=E_ACO4;
    rute=rute4;
else
    [N_idk,it_idk]=find(E_ACO==min(min(E_ACO)));
    E_ACO=E_ACO(N_idk(1),it_idk(1));
    rute=rute(:,:,N_idk(1),it_idk(1));
end
rute=norm_rute2(rute);
rute(sum(rute')==2,:)=[];
end