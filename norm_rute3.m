function rute3=norm_rute3(rute,kendaraan,kota)
[nveh,~]=size(kendaraan);
[ncit,~]=size(kota);
rute3=[ones(nveh,1) zeros(nveh,ncit-1) ones(nveh,1)];
rute=[1 rute];
rute(rute==1)=0;
x=find(rute==0);
for i=1:nveh
    if i==nveh
        x(i+1)=length(rute);
        rute3(i,2:(x(i+1)-x(i)+1))=rute(x(i)+1:x(i+1));
    else
        rute3(i,2:(x(i+1)-x(i)))=rute(x(i)+1:x(i+1)-1);
    end
end
end