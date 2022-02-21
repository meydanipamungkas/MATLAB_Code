function rute1=norm_rute(rute)
rute1=rute;
[panjang,lebar]=size(rute1);
y=zeros(panjang,lebar-1);
for k=1:panjang
    y(k,:)=[diff(rute1(k,:)')'];
    rute1(k,find(y(k,1:end-1)==0)+1)=0;
end
end