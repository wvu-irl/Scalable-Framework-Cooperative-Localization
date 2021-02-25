function [ indx ] = resampleMSV(w, N)
% [ indx ] = resampleMSV(w, N)
% MSV (minimum sampling variance) resampling method for particle filtering.  
% Author: Tiancheng Li,Ref:
% T. Li, The optimal arbitrary-proportional finite-set-partitioning, arXiv.or/abs/1411.6529   
% Also see
% T. Li, G. Villarrubia, S. Sun, J. M. Corchado, J. Bajo.
% Resampling methods for particle filtering: identical distribution, a new method and comparable study, 
% Frontiers of Information Technology & Electronic Engineering, DOI:10.1631/FITEE.1500199

% Input:
%       w    the input weight sequence 
%       N    the desired length of the output sequence(i.e. the desired number of resampled particles)
% Output:
%       indx the resampled index according to the weight sequence

if nargin == 1
   N = length(w);
end
M = length(w);
w = w / sum(w); % if required

% integer parts:
Ns = floor(N*w);
% we draw the Residual part first in order to allocate the space for indx
[~,indx] = sort(N*w - Ns);
% indx = zeros(1, N); % then we don't need this sentence
% Draw the deterministic part:
i = 1;
j = 0;
while j < M
    j = j + 1;
    cnt = 1;
    while cnt <= Ns(j)
        indx(i) = j;
        i = i + 1; cnt = cnt + 1;
    end;
end;
indx(N+1:end) =[];


