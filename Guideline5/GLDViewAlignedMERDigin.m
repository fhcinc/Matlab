function GLDViewAlignedMERDigin(t1)
%Use this function to visualize and measure data alignment between channel 
% 1 of the UE Interface and the real time digital input 1 on the Sync Interface. 
% Assumptions:
% uE Interface : Channel 1, microelectrode recording, sampled at 32 kHz. 
% Sync Interface: Digin 1, TTL input, sampled at 32 kHZ. 
% Data is expected in segment 1, 10 Hz TTL input.  
% Cosmin Serban, July 2023

    N = length(t1.segments);
    eog_delays= zeros(1, N); % holds mean MER-EOG misalignment per segment. 
    segs = [1:1:N];

    for k = 1:N
        
        nseg = k;
        if( isempty( find(segs == nseg)))
            continue;
        end
        sf_mer = t1.segments(nseg).sampling_rate_mer; % MER sampling rate; recorded on the first device.
        sf_lfp = t1.segments(nseg).sampling_rate_lf; 
        % Normalize data to more easily visualize alignment
        mer = t1.segments(nseg).channels(1).continuous; % MER data;
        mer_length = length(mer);
        mer = mer-mean(mer);
        mer = (mer/max(mer));

        ts_mer = double(t1.segments(nseg).start_timestamp_mer);        
        
        % time base for the mer:
        tm = linspace(0, length(mer)/ sf_mer, length(mer) );

        % Detect square waveform edges and their timestamps
        nixmer = find(abs(diff(mer))>0.05);
        nixmer = nixmer(diff(tm(nixmer)) > 0.001);

        figure; hold on; grid on;
        plot(tm, mer);
        %plot( tlf, lfp);
        plot(tm(nixmer), mer(nixmer), '*');
        
        title([ 'Segment ' num2str(nseg)]);

        if(isfield(t1.segments(nseg), 'sync'))
            
            %Re-create digital input signal from timestamps
            if(isfield(t1.segments(nseg).sync, 'rt_timestamps'))
                % Alignment is relative to the first MER timestamp. The +4
                % correction factor is a fixed empirical value.  
                rt_align = double(t1.segments(nseg).sync.rt_timestamps) - ts_mer + 4;
                % Only keep the positive timestamps, because the negative
                % ones correspond to the discarded MER samples.
                rt_align = rt_align(rt_align >= 0);
                
                ts_rt = rt_align/sf_mer;
                digin = zeros(1, length(mer_length));
                state = 0;
                for i = 1:length(rt_align)-1
                    digin( rt_align(i):rt_align(i+1)) = mod(state+1, 2);
                    state = state+1;
                end
                % Re-create digital input signal. 
                t1.segments(nseg).sync.digin = digin; 
                t1.segments(nseg).sync.rt_timestamps = linspace(0, length(digin)/sf_mer, length(digin));
            end            
            
            if(isfield(t1.segments(nseg).sync, 'digin'))
                plot( t1.segments(nseg).sync.rt_timestamps, t1.segments(nseg).sync.digin, '-*'); 
                legend('MER', 'MER-EDGE', 'DIGIN');
            else
                                   
            end
        else
            legend('MER','MER-EDGE');
        end                
        xlabel( 'Seconds');
        ylabel( 'Normalized amplitude');
        ylim( [-2.5 1.5] );
        
        if(nseg == 1)
            eog_delays = mean(nixmer - rt_align(1:length(nixmer)) );

            figure; hold on;
            title('Delays');
            stem(abs(1e6*(eog_delays/sf_mer)));
            ylabel( 'us');
            xlabel('Segment');
            fprintf('\nmean delay = %f', 1e6*(mean(eog_delays/sf_mer)));
        end
    end
    
end