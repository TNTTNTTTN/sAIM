T = readtable('log000_vehicle_gps_position_0.csv');
sz = size(T);
T_d(:,1:5) = T{:,1:5};

for n = 1:sz(1,1)
    T_dt(n,1) = 0;
    T_dt(n,2) = 0;
    T_dt(n,3) = T_d(n,1) / 10^6;
    T_dt(n,4) = T_d(n,3) / 10^7;
    T_dt(n,5) = T_d(n,4) / 10^7;
    T_dt(n,6) = T_d(n,5) / 10^3;
    T_dt(n,7:12) = time2cal(T{n,2}*1E-06);
    T_dt(n,10) = T_dt(n,10) + 9;
end

T_dname(1,:) = ["자동,수동"; "경로점"; "GPS Time"; "Latitude"; "Longitude"; "Altitude"; "Time Stamp"; "0"; "0"; "0"; "0"; "0"];
T_total = vertcat(T_dname,T_dt);
save('Flight_Log.xls', 'T_dt', '-ascii');

function [cal] = time2cal(time)

cal  = datevec(floor(time ./ 86400) + datenum([1970, 1, 1]));
sec  = (time - cal2time(cal));
hour = floor(sec ./ 3600);
min  = floor((sec - hour * 3600) ./ 60);
sec  = sec - hour * 3600 - min * 60;

cal(:,4) = hour;
cal(:,5) = min;
cal(:,6) = sec;

end
    
 function [time] = cal2time(dt)
% ==============================================================================
% CAL2TIME Convert calendar date and time to standard 
% ---------------------------------------------------------------time---------------
% Inputs:
%   dt <n x 6 matrix> : Calendar date and time (year/month/day/hour/min/sec)
% ------------------------------------------------------------------------------
% Outputs:
%   time <n x 1 matrix> : Standard time [s]
% ------------------------------------------------------------------------------
% References:
%   -
% ------------------------------------------------------------------------------
% Author:
%   Cheolsoon Lim (csleem@sju.ac.kr)
% ------------------------------------------------------------------------------
% History:
%   -
% ==============================================================================

if (size(dt, 2) > 6 || size(dt, 2) < 3)
    time = 0;
else    
    time = (datenum(dt(:,1:3)) - datenum([1970, 1, 1])) * 86400;
    time = time + dt(:,4)*3600 + dt(:,5)*60 + dt(:,6);
end
 end