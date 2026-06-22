function M = metrics(out)
%METRICS  Tracking-performance metrics from a closed-loop simulation.
%   M.RMSE/IAE/peakErr tracking metrics; rRefSS/rActSS steady-state checks.
    ls = out.logsout;
    ref = getSignalExact(ls, 'yawRateRef');
    act = getSignalExact(ls, 'yawRate');
    tRef = ref.Values.Time;   yRef = ref.Values.Data;
    tAct = act.Values.Time;   yAct = act.Values.Data;
    yActI = interp1(tAct, yAct, tRef, 'linear', 'extrap');
    e = yRef - yActI;
    M.RMSE    = sqrt(mean(e.^2));
    M.IAE     = trapz(tRef, abs(e));
    M.peakErr = max(abs(e));
    M.rRefSS  = yRef(end);
    M.rActSS  = yActI(end);
end

function s = getSignalExact(ls, wantName)
%GETSIGNALEXACT  Fetch a logged signal by EXACT cleaned name.
    for k = 1:ls.numElements
        nmClean = erase(erase(string(ls{k}.Name), '<'), '>');
        if strcmp(nmClean, wantName)
            s = ls{k}; return;
        end
    end
    error('metrics:signalNotFound', 'Logged signal "%s" not found.', wantName);
end
