function GLDViewAligned(t1, t2)
%Call this function after running gld_align to visualize the aligned data. 
% Assumptions:
% uE Interface : Channel 1, microelectrode recording, sampled at 32 kHz. 
% Lf Interface : Channel 1, EOG channel, sampled at 1 kHz. 
% Sync Interface: Digin 1, TTL input, sampled at 32 kHZ. 

    N = length(t1.segments);
    eog_delays= zeros(1, N); % holds mean MER-EOG misalignment per segment. 
    segs = [1:1:N];
    %segs = [9];
    
    for k = 1:N
        
        nseg = k;
        if( isempty( find(segs == nseg)))
            continue;
        end
        sf_mer = t1.segments(nseg).sampling_rate_mer; % MER sampling rate; recorded on the first device.
        sf_eog = t2.segments(nseg).sampling_rate_lf; % EOG data sampling rate; recorded on the second device. 
        sf_lfp = t1.segments(nseg).sampling_rate_lf; 
        % Normalize data to more easily visualize alignment
        mer = t1.segments(nseg).channels(1).continuous; % MER data;
        mer = mer-mean(mer);
        mer = mer/max(mer);

        eog = t2.segments(nseg).channels(1).LF;
        eog = eog-mean(eog);
        eog = eog/max(eog);

        % time base for the mer:
        tm = linspace(0, length(mer)/ sf_mer, length(mer) );

        teog = linspace(0, length(eog)/ sf_eog, length(eog));
        
        nixmer = find(abs(diff(mer))>0.25);
        nixmer = nixmer(diff(tm(nixmer)) > 0.001);

        nixeog = find(abs(diff(eog))>0.25);
        nixeog = nixeog(diff(teog(nixeog)) > 0.002);

        if(length(tm(nixmer)) >=10 && length(teog(nixeog))>=10)
            fprintf('\ndiff (MER-EOG) = %f', tm(nixmer(1:10)) - teog(nixeog(1:10)));

            fprintf('\n(MER-EOG): nseg = %d, mean = %f', nseg, mean(tm(nixmer(1:10)) - teog(nixeog(1:10))));
            eog_delays(nseg) = mean(tm(nixmer(1:10)) - teog(nixeog(1:10)));
        end

        figure; hold on; grid on;
        plot(tm, mer);
        plot(teog, eog);            
        %plot( tlf, lfp);
        plot(tm(nixmer), mer(nixmer), '*');
        plot(teog(nixeog), eog(nixeog), 'o');
        
        title([ 'Segment ' num2str(nseg)]);

        if(isfield(t1.segments(nseg), 'sync'))
            if(isfield(t1.segments(nseg).sync, 'digin'))
                plot( t1.segments(nseg).sync.rt_timestamps, t1.segments(nseg).sync.digin, '-*'); 
                legend('MER', 'EOG',  'MER-EDGE', 'EOG-EDGE', 'DIGIN');
            else
                legend('MER', 'EOG',  'MER-EDGE', 'EOG-EDGE');                    
            end
        else
            legend('MER', 'EOG',  'MER-EDGE', 'EOG-EDGE');
        end                
        xlabel( 'Seconds');
        ylabel( 'Normalized amplitude');
        ylim( [-2.5 1.5] );
    end
    figure; hold on;
    title('EOG delays');
    stem(1000*abs(eog_delays));
    ylabel( 'ms');
    xlabel('Segment');
    fprintf('\nmean delay = %f', mean(eog_delays));
end