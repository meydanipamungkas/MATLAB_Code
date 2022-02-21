function rute4=norm_rute4(tau,xy,kendaraan,kota)
[ncit,~]=size(kota);
[nveh,~]=size(kendaraan);
rute4=zeros(nveh,ncit+1);
rute4(:,[1 end])=1;
i=1;
j=1;
idk=1;
tau(:,1,:)=0;
while j<=nveh
    while i<=ncit-1
        if sum(sum(tau(:,:,j)))==0
            i=ncit;
        else
            [~,idk]=max(tau(idk,:,j));
            rute4(j,i+1)=idk;
            [~,~,~,~,infeasibility,~,~,~,~]=feasibilitas(xy,rute4,kendaraan,kota);
            if infeasibility>0
                rute4(j,i+1)=0;
                tau(:,idk,j)=0;
            else
                tau(:,idk,:)=0;
                i=i+1;
            end
        end
    end
    if j==1
        i=min(find(rute4(j,2:end)==0));
    else
        idk_i=max(find(rute4(j,2:ncit)~=0));
        i=idk_i+1;
    end
    j=j+1;
end
end