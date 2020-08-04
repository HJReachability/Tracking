function TVTEB = getTVTEB(tt,TEB,tau,TsWS,N)
% Obtain a list of time-varying TEB

    % time-stamped TEB
    timeTEB = [];
    for i = 0:length(TEB)-1
        timeTEB = [timeTEB i*tau];
    end
    TEB = [timeTEB; TEB]';

    % Create time sequence corresponding to MPC, starting from current time
    timeSeq = 0:N;
    timeSeq = tt + timeSeq'*TsWS;

    % Get TVTEB
    TVTEB = [];
    for i = 1:length(timeSeq)
        k = find(TEB(:,1)<=timeSeq(i),1,'last');
    %     if i == length(TEB(:,1))
    %         pause
    %     end
        TVTEB = [TVTEB; TEB(k,2)];
    end

    TVTEB = [timeSeq TVTEB];

end