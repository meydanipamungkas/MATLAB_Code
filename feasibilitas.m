function [rute,total_time,s,t,infeasibility,infeas1,infeas2,infeas3,infeas4]=feasibilitas(xy,rute,kendaraan,kota)
d=squareform(pdist(xy));
[nveh,~]=size(kendaraan);
[ncit,~]=size(kota);

% Infeasibility time
%travel time all nodes k
for i=1:ncit
    for j=1:ncit
        for k=1:nveh
            t(i,j,k)=60*d(i,j)/kendaraan(k,3);
        end
    end
end

%travel time each route
time=0;
for k=1:nveh
    for i=1:ncit
        while i<=ncit
            if rute(k,i+1)==0
                rute(k,i+1)=rute(k,i);
            else
                time(k,i)=t(rute(k,i),rute(k,i+1),k);
            end
            i=i+1;
        end
    end
end

%normalisasi rute
rute1=norm_rute(rute);

%serving time
s=zeros(nveh,ncit+1);
h=kota(:,4);
D=kota(:,1);
for k=1:nveh
    for i=1:ncit+1
        if rute1(k,i)==0
            s(k,i)=0;
        else
            s(k,i)=(ceil(D(rute1(k,i))./kendaraan(k,2)))*(2*h(rute1(k,i)).*sign(rute1(k,i))./kendaraan(k,4));
        end
    end
end

%time window
first_tw=reshape(kota(rute(:,:),2),nveh,ncit+1);
arrival=zeros(nveh,ncit+1);
leave=zeros(nveh,ncit+1);
for k=1:nveh
    i=1;
    while i<=ncit+1
        if i==1
            arrival(k,i)=0;
            leave(k,i)=max(first_tw(k,i+1),time(k,i))-time(k,i);
        elseif i==2
            arrival(k,i)=max(first_tw(k,i),time(k,i-1));
            leave(k,i)=arrival(k,i)+s(k,i);
        else
            arrival(k,i)=max(first_tw(k,i),arrival(k,i-1)+s(k,i-1)+time(k,i-1));
            leave(k,i)=arrival(k,i)+s(k,i);
        end
        i=i+1;
    end
end

% Infeasibility capacity
%truck load
load_t=0;
for k=1:nveh
    for i=1:ncit
        if rute1(k,i)==0
            load_t(k,i)=0;
        else
            load_t(k,i)=kota(rute1(k,i),1);
        end
    end
end

% Infeasibility
%infeasibility capacity
infeas1=sum(find(kendaraan(:,1)-sum(load_t')')<0);

%infeasibility time window
last_tw=reshape(kota(rute(:,:),3),nveh,ncit+1);
infeas2=sum(sum(leave-last_tw>0));

%infeasibility serving time
infeas3=sum(sum(s(:,1:ncit)-(kota(:,3)-kota(:,2))'>0));

%infeasibility range for each vehicle
total_time=max(leave')'-min(leave')';
infeas4=sum(find(kendaraan(:,5)-total_time<0));

% Infeasibility
infeasibility=infeas1+infeas2+infeas3+infeas4;

end