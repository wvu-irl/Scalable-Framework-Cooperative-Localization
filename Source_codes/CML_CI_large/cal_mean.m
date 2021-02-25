% cal mean
a = [];
for i=1:128
    a=[a mean(errors.mn(i).p)];
end

mean(a)