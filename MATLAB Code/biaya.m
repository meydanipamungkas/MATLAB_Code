function [rute,E]=biaya(xy,wt,wd,rute,kendaraan,kota,delta)
[nveh,~]=size(kendaraan);
[ncit,~]=size(kota);
alpha=.6;
beta=.1;
gamma=.3;
[rute,~,s,t,~]=feasibilitas(xy,rute,kendaraan,kota);

%decision variables
x=zeros(ncit,ncit,nveh);
for i=1:ncit
    for j=1:ncit
        for k=1:nveh
            x(rute(k,i),rute(k,i+1),k)=1;
        end
    end
end
for i=1:ncit
    for j=1:ncit
        for k=1:nveh
            if i==j
                x(i,j,k)=0;
            end
        end
    end
end

%konsumsi energi truk
cons=sum(sum(x.*t));
[panjang,~]=size(rute);
cons=reshape(cons,panjang,1,1);
biaya1=sum(wt.*cons);

%konsumsi energi drone
biaya2=sum(wd.*sum(s')');

%setup cost set kendaraan
biaya3=sum(delta.*sign(reshape(sum(sum(x)),panjang,1)));

%total biaya
E=alpha*biaya1+beta*biaya2+gamma*biaya3;

end