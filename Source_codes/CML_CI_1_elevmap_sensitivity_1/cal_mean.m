% cal mean
a = [];
for i=1:16
    a=[a mean(errors.mn(i).p)];
end

mean(a)