function [t1, t2] = GLDAlign(merFile, eogFile)
%This script performs data alignment given two Guideline 5 .gld file, 
% where one is configured as uE Interface and the other as Lf Interface. 
% Parameters:
% merFile - Full path to the uE Interface data file. 
% eogFile - Full path to the Lf Interface data file. 
% Return values:
% t1 - data structure holding the uE Interface aligned data. 
% t2 - data structure holding the Lf Interface aligned data. 
%

do_correction = 1;
t1 = GLDFile(merFile);
t2 = GLDFile(eogFile);

    N = length(t1.segments);
    segs = 1:1:N;
    
    for k = 1:N
        
        nseg = k;
        if( isempty( find(segs == nseg)))
            continue;
        end
        
        sf_mer = t1.segments(nseg).sampling_rate_mer; % MER sampling rate; recorded on the first device.
        sf_eog = t2.segments(nseg).sampling_rate_lf; % EOG data sampling rate; recorded on the second device. 
        sf_lfp = t1.segments(nseg).sampling_rate_lf; % EOG data sampling rate; recorded on the second device. 

        % drop very short data trials. 
        nsec = floor(length(t1.segments(nseg).channels(1).continuous)/32000);
        if(nsec<1)
            continue;
        end
        
        ts_mer = double(t1.segments(nseg).start_timestamp_mer);
        ts_eog = double(t2.segments(nseg).start_timestamp_lf);

        % Align MER (uE Interface 1) and LF (Lf Interface) data using timestamps. 
        tsd = ts_mer - ts_eog;
        if(do_correction == 1)
            if(tsd > 0 )
                offset_eog = int32(ceil( (double(tsd)/double(sf_mer)) * double(sf_eog) + 0.5))
                offset_mer = 1;
                t2.segments(nseg).start_timestamp_lf = t2.segments(nseg).start_timestamp_lf + uint32(ceil( (double(offset_eog)/double(sf_eog))*double(sf_mer) + 0.5));
            else
                offset_eog = 1;
                offset_mer = abs(tsd)
                ts_mer = ts_mer + offset_mer;
                t1.segments(nseg).start_timestamp_mer = ts_mer;
            end
            if(offset_mer <= 0)
                offset_mer = 1;
            end
            if(offset_eog <= 0)
                offset_eog = 1;
            end
        else
            offset_eog = 1;
            offset_mer = 1; 
        end
        
        % Align MER channels on the first interface
        mer_length = 0;
        nchannels = length(t1.segments(nseg).channels);
        for j = 1:nchannels
            if(~isempty(t1.segments(nseg).channels(j)) )
                t1.segments(nseg).channels(j).continuous = t1.segments(nseg).channels(j).continuous(int32(offset_mer):end);
                mer_length = length(t1.segments(nseg).channels(j).continuous);
            end
        end

        %Align EOG channels on the second (Lf) interface        
        nEogChannels = length(t2.segments(nseg).channels); % number of EOG channels (on the second device)
        for j = 1:nEogChannels
            if(~isempty( t2.segments(nseg).channels(j).LF ))
                t2.segments(nseg).channels(j).LF = t2.segments(nseg).channels(j).LF(int32(offset_eog):end);
            end
        end
        
        
        %Align LF channels on the first (uE) interface
        ts_lfp = t1.segments(nseg).start_timestamp_lf;
        tsd = ts_mer - ts_lfp;
        if(do_correction == 1)
            if(tsd > 0 )
                offset_lf = int32(ceil( (double(tsd)/double(sf_mer)) * double(sf_lfp)))
            else
                offset_lf = 1;
            end
            if(offset_lf<= 0)
                offset_lf= 1;
            end
        else
            offset_lf= 1;
        end
        nEogChannels = length(t1.segments(nseg).channels); % number of EOG channels (on the first device)
        for j = 1:nEogChannels
            if(~isempty( t1.segments(nseg).channels(j).LF ))
                t1.segments(nseg).channels(j).LF = t1.segments(nseg).channels(j).LF(int32(offset_lf):end);
            end
        end
        
        
        %Re-create digital input signal from timestamps
        if(isfield(t1.segments(nseg), 'sync'))
            if(isfield(t1.segments(nseg).sync, 'rt_timestamps'))
                rt_align = double(t1.segments(nseg).sync.rt_timestamps) - ts_mer + 5;
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
        end        
    end
end
