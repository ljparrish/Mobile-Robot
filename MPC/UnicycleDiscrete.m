function zk1 = UnicycleDiscrete(zk,uk,Ts)

M = 10;
delta = Ts/M;
zk1 = zk;
for ct = 1:M
    zk1 = zk1 + delta*UnicycleContinuous(zk1,uk);
end

