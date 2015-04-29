n = round(10.^(1:0.5:4));

for i = 1:length(n)
    tic;
    for j = 1:5
    ccapt(rand(n(i),2)*40, rand(n(i),2)*40, 0.0001, 5, 0.01);
    end
    t(i) = toc/5;
end

loglog(n,t);
hold on;
loglog(n, n.^3,'r');
