disp("init group...");

if simu.subN == 4
    for i = 1:simu.N
        if i==1
            groups{i} = [1 simu.N 2 3];
        elseif i==simu.N-1
            groups{simu.N-1} = [simu.N-1 simu.N-2 simu.N 1];
        elseif i==simu.N
            groups{simu.N} = [simu.N simu.N-1 1 2];
        else
            groups{i} = [i i-1 i+1 i+2];
        end
    end

elseif simu.subN == 8
    for i = 1:simu.N
        if i==1
            groups{1} = [1 simu.N-2 simu.N-1 simu.N 2 3 4 5];
        elseif i==2
            groups{2} = [2 simu.N-1 simu.N 1 3 4 5 6];
        elseif i==3
            groups{3} = [3 simu.N 1 2 4 5 6 7];
        elseif i==simu.N
            groups{simu.N} = [simu.N simu.N-3 simu.N-2 simu.N-1 1 2 3 4];
        elseif i==simu.N-1
            groups{simu.N-1} = [simu.N-1 simu.N-4 simu.N-3 simu.N-2 simu.N 1 2 3];
        elseif i==simu.N-2
            groups{simu.N-2} = [simu.N-2 simu.N-5 simu.N-4 simu.N-3 simu.N-1 simu.N 1 2];
        elseif i==simu.N-3
            groups{simu.N-3} = [simu.N-3 simu.N-6 simu.N-5 simu.N-4 simu.N-2 simu.N-1 simu.N 1];
        else
            groups{i} = [i i-3 i-2 i-1 i+1 i+2 i+3 i+4];
        end
    end
    
elseif simu.subN == 16
    for i = 1:simu.N
        if i==1
            groups{1} = [1 simu.N-7 simu.N-6 simu.N-5 simu.N-4 simu.N-3 simu.N-2 simu.N-1 simu.N 2 3 4 5 6 7 8];
        elseif i==2
            groups{2} = [2 simu.N-6 simu.N-5 simu.N-4 simu.N-3 simu.N-2 simu.N-1 simu.N 1 3 4 5 6 7 8 9];
        elseif i==3
            groups{3} = [3 simu.N-5 simu.N-4 simu.N-3 simu.N-2 simu.N-1 simu.N 1 2 4 5 6 7 8 9 10];
        elseif i==4
            groups{4} = [4 simu.N-4 simu.N-3 simu.N-2 simu.N-1 simu.N 1 2 3 5 6 7 8 9 10 11];
        elseif i==5
            groups{5} = [5 simu.N-3 simu.N-2 simu.N-1 simu.N 1 2 3 4 6 7 8 9 10 11 12];
        elseif i==6
            groups{6} = [6 simu.N-2 simu.N-1 simu.N 1 2 3 4 5 7 8 9 10 11 12 13];
        elseif i==7
            groups{7} = [7 simu.N-1 simu.N 1 2 3 4 5 6 8 9 10 11 12 13 14];
        elseif i==8
            groups{8} = [8 simu.N 1 2 3 4 5 6 7 9 10 11 12 13 14 15];
        elseif i==simu.N
            groups{simu.N} = [simu.N simu.N-8 simu.N-7 simu.N-6 simu.N-5 simu.N-4 simu.N-3 simu.N-2 simu.N-1 1 2 3 4 5 6 7];
        elseif i==simu.N-1
            groups{simu.N-1} = [simu.N-1 simu.N-9 simu.N-8 simu.N-7 simu.N-6 simu.N-5 simu.N-4 simu.N-3 simu.N-2 simu.N 1 2 3 4 5 6];
        elseif i==simu.N-2
            groups{simu.N-2} = [simu.N-2 simu.N-10 simu.N-9 simu.N-8 simu.N-7 simu.N-6 simu.N-5 simu.N-4 simu.N-3 simu.N-1 simu.N 1 2 3 4 5];
        elseif i==simu.N-3
            groups{simu.N-3} = [simu.N-3 simu.N-11 simu.N-10 simu.N-9 simu.N-8 simu.N-7 simu.N-6 simu.N-5 simu.N-4 simu.N-2 simu.N-1 simu.N 1 2 3 4];
        elseif i==simu.N-4
            groups{simu.N-4} = [simu.N-4 simu.N-12 simu.N-11 simu.N-10 simu.N-9 simu.N-8 simu.N-7 simu.N-6 simu.N-5 simu.N-3 simu.N-2 simu.N-1 simu.N 1 2 3];
        elseif i==simu.N-5
            groups{simu.N-5} = [simu.N-5 simu.N-13 simu.N-12 simu.N-11 simu.N-10 simu.N-9 simu.N-8 simu.N-7 simu.N-6 simu.N-4 simu.N-3 simu.N-2 simu.N-1 simu.N 1 2];
        elseif i==simu.N-6
            groups{simu.N-6} = [simu.N-6 simu.N-14 simu.N-13 simu.N-12 simu.N-11 simu.N-10 simu.N-9 simu.N-8 simu.N-7 simu.N-5 simu.N-4 simu.N-3 simu.N-2 simu.N-1 simu.N 1];
        else
            groups{i} = [i i-8 i-7 i-6 i-5 i-4 i-3 i-2 i-1 i+1 i+2 i+3 i+4 i+5 i+6 i+7];
        end
    end
else
    disp("input wrong subgroup size!");
end