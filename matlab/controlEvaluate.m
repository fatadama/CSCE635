%% controlEvaluate
%
% TDW
% 2016-06-06
% LASR
%
% Load data file. Evaluate control metrics.

clear variables;
close all;

% load data
loadData;

%% evaluate heading control metrics

%find breaks in the time history of controlObj(:,1)
inbr = find(abs(diff(controlObj(:,1)) > 2.0));
inbr = [inbr;length(controlObj)];

for kcount = 1:length(inbr)
    %array of indices to use
    if kcount == 1
        inu = 1:inbr(kcount);
    else
        inu = (inbr(kcount-1)+1):inbr(kcount);
    end
    % interpolate gps for this time
    inr = find ( diff(gps(:,1))==0.0);
    inp = setdiff(1:length(gps),inr);
    gps2 = gps(inp,:);
    gpsi = interp1(gps2(:,1),gps2(:,6),controlObj(inu,1));
    % interpolate the synthetic waypoint log for this time
    ins = find ( diff(synWp(:,1))==0.0);
    inps = setdiff(1:length(synWp),ins);
    sw2 = synWp(inps,:);
    synWpi = interp1(sw2(:,1),sw2(:,2:3),controlObj(inu,1),'previous');
    % find the switching time of the target waypoint
    st = find( sum(diff(synWpi).^2,2)>1e-2 );
    % add the first time as a switching time
    st = [1;st];
    % for each switching time, process the duration, the rise time, and
    % other transient charactersitics
    fprintf('%8s %8s %8s %8s\n','t_rise','t_sttl','t_f','osc mtrc');
    for ki = 1:length(st)
        if ki == length(st)
            rng = (st(ki)+1):length(synWpi);
        else
            rng = (st(ki)+1):st(ki+1);
        end
        % compute error for this loop
        err = minAngleErr(controlObj(rng,5),controlObj(rng,7));
        % determine rise time if appropriate
        if abs(err(1)) > 10*d2r
            % use 5 pct threshold
            thres = 0.05*max(abs(err));
            irise = find( abs(err) <= thres,1,'first');
            trise = controlObj(rng(irise))-controlObj(rng(1));
            % find last time we are greater than the threshols
            isettle = find(abs(err) >= thres,1,'last');
            tsettle = controlObj(rng(isettle))-controlObj(rng(1));
            % threshold the settle time to try to catch if we're still
            % oscillating at the final time
            if tsettle >= 0.98*(controlObj(rng(end))-controlObj(rng(1)))
                tsettle = (controlObj(rng(end))-controlObj(rng(1)));
            end
            % as a metric of oscillation, compute the MSE after the rise time 
            oscMetric = mean(err(irise:end).^2);
            fprintf('%8.3g %8.3g %8.3g %8.3g\n',trise,tsettle,controlObj(rng(end))-controlObj(rng(1)),oscMetric);
        end
        figure;
        subplot(211);
        plot(controlObj(rng,1),minAngleErr(controlObj(rng,5),controlObj(rng,7)),'b-','linewidth',2);
        hold on;
        plot([controlObj(rng(1),1) controlObj(rng(end),1)],0.05*abs(err(1)).*[1 1],'k--');
        plot([controlObj(rng(1),1) controlObj(rng(end),1)],-0.05*abs(err(1)).*[1 1],'k--');
        ylabel('hdg err (rad)');
        subplot(212);
        plot(controlObj(rng,1),controlObj(rng,4),'b-','linewidth',2);
        ylabel('speed (m/s)');
    end
    
%     figure;
%     plot(controlObj(inu,1),minAngleErr(controlObj(inu,5),controlObj(inu,7)),'b-','linewidth',2);
%     hold on;
%     plot(controlObj(inu,1),minAngleErr(gpsi,controlObj(inu,7)),'r--');
%     set(gca,'xlim',[controlObj(inu(1),1) controlObj(inu(end),1)]);
%     grid on;
%     ylabel('heading error (rad)');
%     legend('measured-ref','GPS-ref');
end