function rute2=norm_rute2(rute)
rute2=rute;
[nveh,ncit]=size(rute2);
for i=2:ncit-2
    for j=1:nveh
        if rute2(j,i)==0
            [~,x]=find(rute2(j,i:ncit-1)~=0);
            x=x+i-1;
            rute2(j,[i min(x)])=rute2(j,[min(x) i]);
        end
    end
end
y=find(sum(rute2)==0);
rute2(:,y)=[];
end