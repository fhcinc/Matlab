function [t, ts,ts_lf] = GLDFile(fileName)
% Parses a GL5 file and extracts raw data and event information. 
% The general structure of output t:
%t.segments
%t.segments(n).drive(1).depth - timestamp and drive depth on the left side. 
%t.segments(n).drive(2).depth - timestamp and drive depth on the right side. 
%t.segments(n).offset_ac_mer - AC offset calibration constant
%t.segments(n).offset_dc_mer - DC offset calibration constant
%t.segments(n).start_timestamp_mer - MER start timestamp
%t.segments(n).start_timestamp_lf - LF start timestamp
%t.segments(n).sampling_rate_mer  - MER sampling rate.
%t.segments(n).sampling_rate_lf - LF sampling rate
%t.segments(n).v_cal_mer - Scale raw MER data to microvolts.
%t.segments(n).v_cal_lf - Scale raw LF data to microvolts.
%
%t.segments(n).channels(m)
%t.segments(n).channels(m).continuous - extracted MER data for the segment, in microvolts.
%t.segments(n).channels(m).LF - extracted LFP data for the segment, in microvolts.
%
%t.segments(n).stim_params(k) - structure holding the stimulation parameters.
%t.segments(n).stim_params(k).stim_type: 1 - macro, 0 - micro
%t.segments(n).stim_params(k).stim_mode: 1 - constant voltage, 0 - constant current
%t.segments(n).stim_params(k).output_channel_map: output channel bitmap
%t.segments(n).stim_params(k).return_channel_map: return channel bitmap
%t.segments(n).stim_params(k).pulse - pulse parameters structure
%t.segments(n).stim_params(k).stim_on - stimulation on timestamp
%t.segments(n).stim_params(k).stim_off - stimulation off timestamp
%
% t.segments(nSegment).sync.rt_timestamps - timestamps of the
% real time digital input on the Sync unit. 
%
%t.fileInfo
%t.fileInfo.version - file version
%t.fileInfo.deviceId - device identifier (0 - GL5 port 1 or 1 - GL5 port 2)
%
% motion sensor data:
% t.segments(n).motion: 
%   struct with fields:
%       sampling_rate: 50               % sensor sampling rate
%     start_timestamp: xxx              % closest MER timestamp to the
%                                       % first motion sensor sample. 
%
%                data: [1×1 struct]     % sensor data:
%
%t.segments(1).motion.data
%   struct with fields:
% 
%     timestamps: [1×M int64]           % an array of 64-bit integers that
%                                       % match the binary representation 
%                                       % of a System.DateTime .NET Framework
%                                       % object. 
%
%              X: [1×M single]          % Accelerometer X axis values.
%              Y: [1×M single]          % Accelerometer Y axis values.
%              Z: [1×M single]          % Accelerometer Z axis values.
%
% Cosmin Serban, FHC Inc, 2019
%
%
% TODO: This script does not work well with large files since it loads all
% contents in memory before parsing. While this is more efficient, it may
% lead to Matlab freezing for very large files. Must load chunks of the
% file in memory. Additionaly, one can specify time intervals that can be
% parsed instead of the whole file, as it may be possible that all data
% does not fit the available RAM. The time intevals can be specified by
% means of indexes. Typically, the index file will be much smaller and can
% be loaded first. Then, users can be presented with a time range that they
% can specify to extract the data. 

if(nargin < 1)
    warning('Please specify file name!' );
    return;
end

global gvars;
gvars = [];
gvars.verbose = 1;

if(nargin < 2)
    destFile = 't.mat';
end

% TODO: split_records
split_records = 1; % If set to 1, the records will be split when ADS_EVENT_START_REC event is found.  
mer_split_found = 0;
lfp_split_found = 0;

skip_mer = 0; % does not extract MER data.
skip_lf = 0; % does not extract LF data. 

legacy_codes;
message_codes;

nSegment = 0;
nStimCount=1;
nPulseCount=1;
t = [];
t.segments = struct();

depth_index = ones(1,2);

%Parameters
v_cal_mer = ones(2,8); % MER voltage calibration - two interfaces. 
v_cal_lfs = ones(2,8); %  LFP voltage calibration - two interfaces. 
v_cal_aux = ones(1,2); % voltage calibration - sync interface

gain = 12 * 5.92 * 2.82;
voltage_calibration = (4/(2^23-1))/ gain;

last_ts = 0;
last_length = 0;
last_raw_ts = 0;
last_aux_ts = 0;
last_lfp_ts = 0;
last_lfp_len = 0;
last_aux_len = 0;

fp = fopen(fileName, 'rb');
FileInfo = dir( fileName );
fileSize = FileInfo.bytes;
dataSize = fileSize/ 4;
num_channels = 8;

ts = [];
ts_lf = [];
ch_index = ones(1, num_channels);
ch_index_lf = ones(1, num_channels);
ch_index_motion = ones(1,4); % 3-axis accelerometer
ch_index_aux = ones(1,num_channels);

num_msgs = 1;

mer_channel_list = [];
mer_channel_map = 0;

lf_channel_list = [];
lf_channel_map = 0;

fileData = fread( fp, dataSize, '*uint32', 0, 'n' );
count = 1;

tic
while( count <= size(fileData,1) )

    word = fileData( count );
    count = count + 1;
    
    if (word==SYNC)

        msg_code = uint32(fileData( count ));
        count = count + 1;
        msg_len = fileData( count );
        count = count + 1;
        
        if (msg_code == ADS_DATA_MER  )
            num_msgs = num_msgs + 1;
            
            if(skip_mer == 1)
                count = count + msg_len;
                continue;
            end
            msg_bitmap = fileData( count );
            count = count + 1;
            msg_timestamp = fileData( count );
            count = count + 1;

            num_channels = bitcount(msg_bitmap);
            num_samples = (msg_len-2)/ num_channels;
            msg_seq = bitshift( bitand( msg_bitmap, hex2dec( 'F0000000' )), -28);
            msg_seq = msg_seq + 1;            


            if( last_ts > 0 )
                if( msg_timestamp - last_ts > last_length )
                   %fprintf( '\nMER. data loss, expected: %d, found: %d', last_ts + last_length, msg_timestamp );
                end
                if(msg_timestamp < last_ts)
                    %fprintf( '\nMER. restart record at : %d, from %d', msg_timestamp, last_ts );
                end
                last_ts = msg_timestamp;
                last_length = num_samples;
            else 
                last_ts = msg_timestamp;
                last_length = num_samples;
            end
            
            if( ~isfield(t.segments(nSegment), 'start_timestamp_mer'))
                t.segments(nSegment).start_timestamp_mer = msg_timestamp;
            elseif( isempty(t.segments(nSegment).start_timestamp_mer))
                t.segments(nSegment).start_timestamp_mer = msg_timestamp;
            end
                        
            if(mer_channel_map ~= msg_bitmap)
                mer_channel_list = [];
                for k = 0:7
                    if( 0 ~= bitand( msg_bitmap, bitshift(1,k))) 
                        mer_channel_list = [mer_channel_list k+1];
                    end
                end
                for i = 1:num_channels
                    channel_index = mer_channel_list(i);

                    if( ~isfield(t.segments(nSegment), 'channels'))
                        t.segments(nSegment).channels = [];                            
                        t.segments(nSegment).channels(channel_index).continuous = [];
                    end
                    if(length(t.segments(nSegment).channels)<channel_index)
                        t.segments(nSegment).channels(channel_index).continuous = [];
                    end
                end
                mer_channel_map = msg_bitmap;
            end
            
            if(split_records == 1 )
                if(mer_split_found == 1)
                    for i = 1:num_channels
                        channel_index = mer_channel_list(i);
                        ch_index(channel_index) = 1;
                    end
                end
            end

            try
                if( count + (msg_len-2) <= dataSize )
                    for i = 1:num_channels
                        channel_index = mer_channel_list(i);

                        m_data = typecast( uint32( fileData( count: (count + num_samples -1) ) ), 'int32' );
                        %m_data = fileData( count: (count + num_samples -1) );
                        count = count + num_samples;
                        m_data = t.segments(nSegment).v_cal_mer(channel_index) .* single(m_data);

                        t.segments(nSegment).channels(channel_index).continuous( ch_index(channel_index) : (ch_index(channel_index) + length(m_data) - 1) ) =  m_data;
                        ch_index(channel_index) = ch_index(channel_index) + length( m_data );
                        
                    end
                end
                if(mer_split_found == 1)
                    mer_split_found = 0;
                end                
            catch err 
                fprintf( '\nError at count %d\n', count );
                err.message
                err.cause{ : }
                
                for k = 1:length( err.stack )
                    err.stack(k)
                end
                break;
            end
                        
        elseif(msg_code == LFP_DATA_RAW)
            
            if(skip_lf == 1)
                count = count + msg_len;
                continue;
            end
            
            msg_bitmap = fileData( count );
            count = count + 1;
            msg_timestamp = fileData( count );
            count = count + 1;

            num_channels = bitcount(msg_bitmap);
            num_samples = (msg_len-2)/ num_channels;
            %fprintf( '\nmsg_bitmap = %x', msg_bitmap );
            msg_seq = bitshift( bitand( msg_bitmap, hex2dec( 'F0000000' )), -28);
            msg_seq = msg_seq + 1;            
            
            if(lf_channel_map ~= msg_bitmap)
                lf_channel_list = [];
                for k = 0:7
                    if( 0 ~= bitand( msg_bitmap, bitshift(1,k))) 
                        lf_channel_list = [lf_channel_list k+1];
                    end
                end
                for i = 1:num_channels
                    channel_index = lf_channel_list(i);
                    if( ~isfield(t.segments(nSegment), 'channels'))
                        t.segments(nSegment).channels = [];                            
                        t.segments(nSegment).channels(channel_index).LF = [];
                    end
                    if(length(t.segments(nSegment).channels)<channel_index)
                        t.segments(nSegment).channels(channel_index).LF = [];
                    end

                    if( ~isfield(t.segments(nSegment).channels(channel_index), 'LF'))
                        t.segments(nSegment).channels(channel_index).LF = [];
                    end
                end
                lf_channel_map = msg_bitmap;
            end
            if(split_records == 1 )
                if(lfp_split_found == 1)
                    for i = 1:num_channels
                        channel_index = lf_channel_list(i);
                        ch_index_lf(channel_index) = 1;
                    end
                end
            end

            
            if( last_lfp_ts > 0 )
                if( msg_timestamp - last_lfp_ts > double(( double(t.segments(nSegment).sampling_rate_mer)/double(t.segments(nSegment).sampling_rate_lf))*last_lfp_len)) 
%                      fprintf( '\nLFP data loss, expected: %d, found: %d, dif = %d', ...
%                          last_lfp_ts + 32 * last_lfp_len, msg_timestamp, (last_ts + 32 * last_lfp_len) - msg_timestamp );
                else
                    %fprintf( '\nLFP timestamp OK: last = %d, new: %d', last_ts, msg_timestamp);
                end
                last_lfp_ts = msg_timestamp;
                last_lfp_len = num_samples;

            else 
                %fprintf( '\nfirst_lfp_ts = %d', msg_timestamp );
                last_lfp_ts = msg_timestamp;
                last_lfp_len = num_samples;
                ts_lf = [ ts_lf msg_timestamp];
            end
            
            if( ~isfield(t.segments(nSegment), 'start_timestamp_lf'))
                t.segments(nSegment).start_timestamp_lf = msg_timestamp;
            elseif( isempty(t.segments(nSegment).start_timestamp_lf))
                t.segments(nSegment).start_timestamp_lf = msg_timestamp;
            end

            try
                if( count + (msg_len-2) <= dataSize )
                    for i = 1:num_channels
                        channel_index = lf_channel_list(i);

                        m_data = typecast( uint32( fileData( count: (count + num_samples -1)  ) ), 'int32' );
                        count = count + num_samples;
                        m_data = t.segments(nSegment).v_cal_lf(channel_index) .* single(m_data) - single(t.segments(nSegment).offset_dc_lfs(channel_index));

                        t.segments(nSegment).channels(channel_index).LF( ch_index_lf(channel_index) : (ch_index_lf(channel_index) + length(m_data) - 1) ) =  m_data;
                        ch_index_lf(channel_index) = ch_index_lf(channel_index) + length( m_data );
                    end
                end
                if(lfp_split_found == 1)
                    lfp_split_found = 0;
                end                
                
            catch err 
                fprintf( '\nError at count %d\n', count );
                err.message
                err.cause{ : }
                
                for k = 1:length( err.stack )
                    err.stack(k)
                end
                break;
            end     
        elseif(msg_code == MOTION_DATA_MESSAGE)
            msg_bitmap = fileData( count );
            count = count + 1;
            msg_timestamp = fileData( count );
            count = count + 1;
            
            num_channels = bitcount(msg_bitmap);
            num_samples = (msg_len-2)/(num_channels+1);
            %fprintf( '\nmsg_bitmap = %x', msg_bitmap );
            msg_seq = bitshift( bitand( msg_bitmap, hex2dec( 'F0000000' )), -28);
            msg_seq = msg_seq + 1;            
            
            if( ~isfield(t.segments(nSegment).motion, 'start_timestamp'))
                t.segments(nSegment).motion.start_timestamp = last_ts;
            end
            
            try
                if( count + (msg_len-2) <= dataSize )
                    if(num_channels == 4)
                        for i = 1:num_channels
                            if(i==1)
                                for k = 1:num_samples
                                    d1 = int64(fileData(count));
                                    count = count+1;
                                    d2= (fileData(count));
                                    count = count+1;
                                    % The data is stored in little endian format. 
                                    % Must confirm that this holds on other machines if the file
                                    % format is preserved. 
                                    start_date = int64( bitor( bitshift(int64(d2), 32), int64(d1)));
                                    t.segments(nSegment).motion.data.timestamps( ch_index_motion(i)) = start_date;
                                    ch_index_motion(i)= (ch_index_motion(i) + 1);
                                end
                                
                            elseif(i==2)
                                m_data = typecast( uint32( fileData( count: (count + num_samples -1)  ) ), 'single' );
                                t.segments(nSegment).motion.data.X( ch_index_motion(i): (ch_index_motion(i) + length(m_data)-1)) = m_data;
                                ch_index_motion(i)= (ch_index_motion(i) + length(m_data));
                                count = count + num_samples;
                            elseif(i==3)
                                m_data = typecast( uint32( fileData( count: (count + num_samples -1)  ) ), 'single' );
                                t.segments(nSegment).motion.data.Y( ch_index_motion(i): (ch_index_motion(i) + length(m_data)-1)) = m_data;
                                ch_index_motion(i)= (ch_index_motion(i) + length(m_data));
                                count = count + num_samples;
                            elseif(i==4)
                                m_data = typecast( uint32( fileData( count: (count + num_samples -1)  ) ), 'single' );
                                t.segments(nSegment).motion.data.Z( ch_index_motion(i): (ch_index_motion(i) + length(m_data)-1)) = m_data;
                                ch_index_motion(i)= (ch_index_motion(i) + length(m_data));
                                count = count + num_samples;
                            end
                        end
                    end
                end
            catch err 
                fprintf( '\nError at count %d\n', count );
                err.message
                err.cause{ : }
                
                for k = 1:length( err.stack )
                    err.stack(k)
                end
                break;
            end     

        elseif (msg_code == SYNC_INT_INPUT_DATA  )
                
            msg_bitmap = fileData( count );
            count = count + 1;
            msg_timestamp = fileData( count );
            count = count + 1;

            msg_seq = bitshift( bitand( msg_bitmap, hex2dec( 'F0000000' )), -28);
            msg_seq = msg_seq + 1;            

            num_channels = bitcount(msg_bitmap);
            num_samples = (msg_len-2)/ num_channels;
            %fprintf( '\nmsg_bitmap = %x', msg_bitmap );
            
            if( last_aux_ts > 0 )
                if( msg_timestamp - last_aux_ts > (last_aux_len*4) )
                    %fprintf( '\nAUX data loss, expected: %d, found: %d', last_aux_ts + last_aux_len*4, msg_timestamp );
                end
                %ts = [ ts msg_timestamp ];
                last_aux_ts = msg_timestamp;
                %fprintf( '\nlast_ts = %d', msg_timestamp );
            else 
                fprintf( '\nfirst aux ts = %d', msg_timestamp );
                last_aux_ts = msg_timestamp;
            end
            last_aux_len = msg_len;
            %fprintf( '\nnum_channels = %d', num_channels );
            try
                
                for i = 1:num_channels
                    if( count + num_samples <= dataSize )
                        m_data = typecast( uint32( fileData( count: (count + num_samples -1) ) ), 'single' );
                        count = count + num_samples;
                        %m_data = voltage_calibration .* m_data;

                        %if( isfield( t(i).channels, 'continuous' ) )
                            %t(i).channels.continuous = cat(1,t(i).channels.continuous, m_data);
                        t.segments(nSegment).aux.channels(i).continuous( ch_index_aux(i) : (ch_index_aux(i) + length(m_data) - 1) ) =  m_data;
                        ch_index_aux(i) = ch_index_aux(i) + length( m_data );
                        %else 
                        %    t(i).channels.continuous = m_data;
                        %end
                    end
                end
                
                if(~isfield(t.segments(nSegment).aux, 'timestamps_aux'))
                    t.segments(nSegment).aux.timestamps_aux = msg_timestamp; 
                else
                    t.segments(nSegment).aux.timestamps_aux = [t.segments(nSegment).aux.timestamps_aux msg_timestamp]; 
                end                
            catch err 
                fprintf( '\nError at count %d\n', count );
                err.message
                err.cause{ : }
                
                for k = 1:length( err.stack )
                    err.stack(k)
                end
                break;
            end            
        elseif(msg_code == MOTION_SENSOR_EVENT_SAMPLING_RATE)
            sampling_rate = fileData(count);
            count = count + 1;
            t.segments(nSegment).motion.sampling_rate = sampling_rate;
            
        elseif(msg_code == SYNC_INT_EVENT_DIG_INPUT)
            deviceId = fileData(count);
            count = count + 1;
            timestamp = fileData(count); 
            count = count + 1;
            port0 = fileData(count);
            count = count + 1;
            port1 = fileData(count);
            count = count + 1;
            port5 = fileData(count);
            port5 = bitand(port5, 1);
            count = count + 1;

            if( ~isfield( t(deviceId + 1).device, 'sync' ) )
                t.segments(nSegment).sync.timestamps = timestamp;
                t.segments(nSegment).sync.port1 = port0;
                t.segments(nSegment).sync.port2 = port1;
                t.segments(nSegment).sync.digin = port5;        
            else
                t.segments(nSegment).sync.timestamps = [ t.segments(nSegment).sync.timestamps timestamp];
                t.segments(nSegment).sync.port1 = [ t.segments(nSegment).sync.port1 port0];
                t.segments(nSegment).sync.port2 = [ t.segments(nSegment).sync.port2 port1];
                %t.segments(nSegment).sync.digin = [ t(deviceId + 1).device.sync.digin port5];        
            end
            
        elseif(msg_code == SYNC_INT_EVENT_REALTIME_DIG_INPUT)
            deviceId = fileData(count);
            count = count + 1;
            ntimestamps = msg_len -1;
            for i = 1:ntimestamps
                timestamp = fileData(count);
                count = count + 1;
                if( ~isfield( t.segments(nSegment), 'sync' ) )
                    t.segments(nSegment).sync.rt_timestamps = timestamp;
                else
                    if( ~isfield( t.segments(nSegment).sync, 'rt_timestamps' ) )
                        t.segments(nSegment).sync.rt_timestamps = timestamp;
                    else
                        t.segments(nSegment).sync.rt_timestamps = [ t.segments(nSegment).sync.rt_timestamps timestamp];        
                    end
                end
            end
        elseif(msg_code == SYNC_INT_EVENT_SAMPLING_RATE)
            sampling_rate = fileData(count);
            count = count + 1;
            if( ~isfield( t.segments(nSegment), 'aux' ) )
                t.segments(nSegment).aux.sampling_rate = sampling_rate;
            else
                t.segments(nSegment).aux.sampling_rate = sampling_rate;        
            end
            
        elseif( msg_code == STIM_EVENT_STIM_ON )
            deviceId =fileData(count);
            count = count + 1;
            stim_on_ts = fileData(count);
            count = count + 1;

            if( ~isfield( t.segments(nSegment), 'stim_params'))
                t.segments(nSegment).stim_params = [];                
            end
               t.segments(nSegment).stim_params(nStimCount).stim_on = stim_on_ts; 
                
            fdisp( '\nStim On TS = %d', stim_on_ts );
            fdisp( '\nStim count TS = %d', nStimCount );
            
        elseif( msg_code == STIM_EVENT_STIM_OFF )
            deviceId =fileData(count);
            count = count + 1;
            stim_off_ts = fileData(count);
            count = count + 1;
                        
            if( ~isfield( t.segments(nSegment), 'stim_params'))
                t.segments(nSegment).stim_params = [];
            end
            t.segments(nSegment).stim_params(nStimCount).stim_off = stim_off_ts;
            nStimCount=nStimCount+1;
            nPulseCount=1;
            fdisp( '\nStim Off TS = %d', stim_off_ts );
            
        elseif(msg_code == STIM_EVENT_STIM_TYPE)
           device_id = fileData(count);
           count = count + 1;
           stim_type = fileData(count); % 0 - microstimulation, 1 - macrostimulation
           count = count + 1;           
           
           if( ~isfield( t.segments(nSegment), 'stim_params'))
                t.segments(nSegment).stim_params = [];
           end
           t.segments(nSegment).stim_params(nStimCount).stim_type = stim_type;
           fdisp( 1, '\nDevice %d: stimulation type: 0x%x', device_id, stim_type);
           
        elseif(msg_code == STIM_EVENT_OPERATING_MODE)
           device_id = fileData(count);
           count = count + 1;
           stim_mode = fileData(count); % 0 - Constant Current, 1 - Constant Voltage
           count = count + 1;           
           
           if( ~isfield( t.segments(nSegment), 'stim_params'))
                t.segments(nSegment).stim_params = [];
           end
           t.segments(nSegment).stim_params(nStimCount).stim_mode = stim_mode;
           fdisp( 1, '\nDevice %d: stimulation type: 0x%x', device_id, stim_mode);
            
        elseif(msg_code == STIM_EVENT_CUSTOM_WAVEFORM)
            waveform_id = [];
            for i = 1:4
                waveform_id = [ waveform_id bitshift( bitand(uint32(fileData(count)),uint32(hex2dec('FF000000'))), -24)];
                waveform_id = [ waveform_id  bitshift( bitand(uint32(fileData(count)),uint32(hex2dec('00FF0000'))), -16)];
                waveform_id = [ waveform_id bitshift( bitand(uint32(fileData(count)),uint32(hex2dec('0000FF00'))), -8)];
                waveform_id = [ waveform_id bitand(uint32(fileData(count)),uint32(hex2dec('000000FF')))];
                count = count+1;
            end
            
            if( ~isfield( t.segments(nSegment), 'stim_params'))
                t.segments(nSegment).stim_params = [];
            end
            t.segments(nSegment).stim_params(nStimCount).identifier = waveform_id;
            fdisp('\nWaveId = %d', waveform_id);          
        elseif(msg_code == STIM_EVENT_PULSE_FREQUENCY)
           mdata = fileData(count: (count + msg_len -1));
           count = count + msg_len;        
           device_id = mdata(1);           
           output_channel = mdata(2)+1;
           frequency = mdata(3);
           if( ~isfield( t.segments(nSegment), 'stim_params'))
                t.segments(nSegment).stim_params = [];
           end
           %t.segments(nSegment).stim_params(nStimCount).pulse_freq = frequency;
           
            if(isfield(t.segments(nSegment).stim_params(nStimCount),'pulse'))
                if(size(t.segments(nSegment).stim_params(nStimCount).pulse)==nPulseCount)
                    if(~isempty(t.segments(nSegment).stim_params(nStimCount).pulse(nPulseCount).freq))
                     nPulseCount=nPulseCount+1;
                    end
                end
            end
           t.segments(nSegment).stim_params(nStimCount).pulse(nPulseCount).freq = frequency;
           t.segments(nSegment).stim_params(nStimCount).pulse(nPulseCount).channel=output_channel;
           
           fdisp( 1, '\nDevice %d: channel %d, stimulation frequency: %d', ...
               device_id, output_channel, frequency);

        elseif(msg_code== STIM_EVENT_PULSE_AMPLITUDE)
           mdata = fileData(count: (count + msg_len -1));
           count = count + msg_len;
           device_id = mdata(1);
           output_channel = mdata(2)+1;
           amplitude = typecast( mdata(3), 'single');
           if( ~isfield( t.segments(nSegment), 'stim_params'))
                t.segments(nSegment).stim_params = [];
           end
           %t.segments(nSegment).stim_params(nStimCount).pulse_amplitude = amplitude;
           t.segments(nSegment).stim_params(nStimCount).pulse(nPulseCount).amplitude = amplitude;
           fdisp( 1, '\nDevice %d: channel %d, stimulation pulse amplitude: %d', ...
               device_id, output_channel, amplitude);

        elseif(msg_code == STIM_EVENT_PULSE_DURATION)
           mdata = fileData(count: (count + msg_len -1));
           count = count + msg_len;
           device_id = mdata(1);
           output_channel = mdata(2)+1;
           duration = mdata(3);
           if( ~isfield( t.segments(nSegment), 'stim_params'))
                t.segments(nSegment).stim_params = [];
           end
           %t.segments(nSegment).stim_params(nStimCount).pulse_duration = duration;
           t.segments(nSegment).stim_params(nStimCount).pulse(nPulseCount).duration = duration;
           fdisp( 1, '\nDevice %d: channel %d, stimulation pulse duration (ticks): %d', ...
               device_id, output_channel, duration);
        elseif(msg_code == STIM_EVENT_PULSE_PHASE)
           mdata = fileData(count: (count + msg_len -1));
           count = count + msg_len;
           device_id = mdata(1);
           output_channel = mdata(2)+1;
           phase = mdata(3); % 0 - monophasic, 1-biphasic
           if( ~isfield( t.segments(nSegment), 'stim_params'))
                t.segments(nSegment).stim_params = [];
           end
           %t.segments(nSegment).stim_params(nStimCount).pulse_phase = phase;
           t.segments(nSegment).stim_params(nStimCount).pulse(nPulseCount).phase = phase;
           fdisp( 1, '\nDevice %d: channel %d, stimulation pulse phase (ticks): %d', ...
               device_id, output_channel, phase);

        elseif(msg_code == STIM_EVENT_PULSE_POLARITY)
           mdata = fileData(count: (count + msg_len -1));
           count = count + msg_len;
           device_id = mdata(1);
           output_channel = mdata(2)+1;
           polarity = mdata(3); % 0 - negative, 1 - positive
           if( ~isfield( t.segments(nSegment), 'stim_params'))
                t.segments(nSegment).stim_params = [];
           end
           %t.segments(nSegment).stim_params(nStimCount).pulse_polarity = polarity;
           t.segments(nSegment).stim_params(nStimCount).pulse(nPulseCount).polarity = polarity;
           fdisp( 1, '\nDevice %d: channel %d, stimulation pulse polarity: %d', ...
               device_id, output_channel, polarity);
           
        elseif(msg_code == STIM_EVENT_OUTPUT_CHANNELS)
           mdata = fileData(count: (count + msg_len -1));
           count = count + msg_len;
           device_id = mdata(1);
           output_channel = mdata(2);
           channel_map = mdata(3);
           if( ~isfield( t.segments(nSegment), 'stim_params'))
                t.segments(nSegment).stim_params = [];
           end
           t.segments(nSegment).stim_params(nStimCount).output_channel_map = channel_map;
           fdisp( 1, '\nDevice %d: channel %d, stimulation output channels (hex map): 0x%x', ...
               device_id, output_channel, channel_map);                

        elseif(msg_code == STIM_EVENT_RETURN_CHANNELS)
           mdata = fileData(count: (count + msg_len -1));
           count = count + msg_len;
           device_id = mdata(1);
           output_channel = mdata(2);
           channel_map = mdata(3);
           if( ~isfield( t.segments(nSegment), 'stim_params'))
                t.segments(nSegment).stim_params = [];
           end
           t.segments(nSegment).stim_params(nStimCount).return_channel_map = channel_map;
           fdisp( 1, '\nDevice %d: channel %d, stimulation return channels (hex map): 0x%x', ...
               device_id, output_channel, channel_map); 
                            

        elseif(msg_code == ADS_EVENT_CLASSIFICATION)

            mdata = fileData(count: (count + msg_len -1));
            count = count + msg_len;
            
            position = mdata(1:2); % 64-bit position in file.  
            channel = mdata(3);    % channel number
            guid = mdata(4:end);   % event guid. 
            
            %fdisp(1, '\nEvent at channel %d, ts = %d', channel, last_ts);
        
            
        elseif(msg_code == HMS_EVENT_BOARD_TYPE)
            m_board_type = fileData(count); 
            count = count + 1;
            m_device_id = fileData(count); 
            count = count + 1;
            t.device.board_type = m_board_type;
            fprintf( 1,'\nReceived board type = %d, device id = %d', m_board_type, m_device_id);

%         % ---------------------------------------------------------
        elseif(msg_code == HMS_EVENT_MER_VOLTAGE_CALIBRATION)
            m_channel = fileData(count); 
            count = count + 1;
            m_device_id = fileData(count); 
            count = count + 1;
            m_vcal = fileData(count); 
            count = count + 1;
            m_vcal = typecast(m_vcal, 'single');
            
            offset_dc = fileData(count);
            offset_dc = typecast(offset_dc, 'int32');
            count = count + 1;
            offset_ac = fileData(count);
            offset_ac = typecast(offset_ac, 'int32');
            count = count + 1;
            
            %fprintf(1, '\nMER voltage calibration on device %d, channel %d = %f', ...
            %    m_device_id, m_channel, m_vcal);
            if(m_device_id >= 0 && m_device_id < size(v_cal_mer,1))
                if(m_channel >= 0 && ( (m_channel) < size(v_cal_mer,2)))
                    t.segments(nSegment).v_cal_mer(m_channel+1) = m_vcal;
                    t.segments(nSegment).offset_dc_mer(m_channel+1) = offset_dc;
                    t.segments(nSegment).offset_ac_mer(m_channel+1) = offset_ac;                    
                else
                    fprintf(1,'\nMER channel out of range %d', m_channel);
                end
            else
                fprintf(1,'\nMER device id out of range %d', m_device_id);
            end                    
        % ---------------------------------------------------------
        elseif(msg_code == HMS_EVENT_LFP_VOLTAGE_CALIBRATION)
            m_channel = fileData(count); 
            count = count + 1;
            m_device_id = fileData(count); 
            count = count + 1;
            m_vcal = fileData(count); 
            count = count + 1;
            m_vcal = typecast(m_vcal, 'single');
            
            offset_dc = fileData(count);
            count = count + 1;
            offset_dc = typecast(offset_dc, 'int32');
            %fprintf(1, '\nLFP voltage calibration on device %d, channel %d = %f', ...
            %    m_device_id, m_channel, m_vcal);                    
            if(m_device_id >= 0 && m_device_id < size(v_cal_lfs,1))
                if(m_channel >= 0 && ( (m_channel) <= size(v_cal_lfs,2)))
                    t.segments(nSegment).v_cal_lf(m_channel+1) = m_vcal;
                    t.segments(nSegment).offset_dc_lfs(m_channel+1) = offset_dc;
                else
                    fprintf(1,'\nLF channel out of range %d', m_channel);
                end
            else
                fprintf(1,'\nLF device id out of range %d', m_device_id);
            end   
        elseif(msg_code == EVENT_FILE_HEADER)
            file_type = fileData(count); 
            count = count + 1;
            
            version = fileData(count); 
            count = count + 1;
            deviceId= fileData(count);
            count = count+1;
            
            %start_date = int64(0);
            d1 = int64(fileData(count));
            count = count+1;
            d2= (fileData(count));
            count = count+1;
            % The data is stored in little endian format. 
            % Must confirm that this holds on other machines if the file
            % format is preserved. 
            start_date = int64( bitor( bitshift(int64(d2), 32), int64(d1)));
            
            index1 = int64(fileData(count));
            count = count+1;
            index2= fileData(count);
            count = count+1;
            index = int64( bitor( bitshift(int64(index2), 32), int64(index1)));
            
           % d = datenum(start_date);
            %fdisp('\nfileType = 0x%x, version = %d, device = %d', file_type, version, deviceId);

            t.fileInfo.version = version;
            t.fileInfo.deviceId = deviceId;
            t.fileInfo.start_time = System.DateTime.FromBinary(int64(start_date));
           % t.fileInfo.dateStr = string(t.fileInfo.start_time.ToString);            
            %t.fileInfo.Index = index;
            
        elseif(msg_code == HMS_EVENT_MER_SAMPLING_RATE)
            deviceId = fileData(count);
            count = count + 1;
            samplingRate = single(fileData(count));
            count = count + 1;
            fdisp('\nMER sampling rate = %.0f', samplingRate);
            if(deviceId >= 0 && deviceId < 2)
                if(~isfield(t.segments(nSegment), 'sampling_rate_mer'))
                    t.segments(nSegment).sampling_rate_mer = samplingRate; 
                else
                    % Maintain a list of sample rate changes. 
                    if( isempty( find( (t.segments(nSegment).sampling_rate_mer == samplingRate), 1 ) ) )
                        t.segments(nSegment).sampling_rate_mer = [ t.segments(nSegment).sampling_rate_mer samplingRate]; 
                    end
                end
            end
        elseif(msg_code == HMS_EVENT_LFS_SAMPLING_RATE)
            deviceId = fileData(count);
            count = count + 1;
            samplingRate = single(fileData(count));
            count = count + 1;
            fdisp('\nLF sampling rate = %.0f', samplingRate);
            if(deviceId >= 0 && deviceId < 2)
                if(~isfield(t.segments(nSegment), 'sampling_rate_lf'))
                    t.segments(nSegment).sampling_rate_lf = samplingRate; 
                else
                    % Maintain a list of sample rate changes. 
                    if( isempty( find( (t.segments(nSegment).sampling_rate_lf == samplingRate), 1 ) ) )
                        t.segments(nSegment).sampling_rate_lf = [ t.segments(nSegment).sampling_rate_lf samplingRate]; 
                    end
                end
            end
        elseif(msg_code == ADS_EVENT_START_REC)
            fdisp(1, '\nStart recording');
            data = fileData(count);
            count = count + 1;
            mer_split_found = 1;
            lfp_split_found = 1;
            nSegment = nSegment + 1;
            nStimCount=1;
            nPulseCount=1;
            depth_index = ones(1,2); % reset depth indexes.
            ch_index_motion = ones(1,4); % reset motion sensor indexes.
            ch_index_aux = ones(1,8);
        elseif(msg_code == ADS_EVENT_STOP_REC)
            fdisp(1, '\nStop recording');
            if(count + 1 < size(fileData,1))
                data = fileData(count);
                count = count + 1;     
            end
        elseif(msg_code == CPM_TEMP_EVENT_LOCAL)
            %Extract temperature data, in degrees Celsius, not stored. 
            data = fileData(count);
            count = count + 1;     
        elseif(msg_code == MTC_EVENT_DRIVE_POSITION)
            side = fileData(count);
            count = count + 1;
            position = fileData(count);
            count = count + 1;                 

            next_depth1 = fileData(count);
            count = count + 1;                 
            next_depth2 = fileData(count);
            count = count + 1;                 

            hemi = 'none';
            if(side == 1)
                hemi = 'left';
            elseif(side == 2)
                hemi = 'right';
            end
            t.segments(nSegment).drive(side).depth( depth_index(side),1) = last_ts;
            t.segments(nSegment).drive(side).depth( depth_index(side),2) = position;            
            depth_index(side) = depth_index(side) + 1;
            
            fdisp(1, '\n side = %s, position = %d um', hemi, position); 
        else 
            fdisp('\nUnknown code 0x%x', msg_code);
            count = count + msg_len;
        end
    elseif ( count <= size(fileData,1) )
        fprintf( 1, '\nOut of SYNC, %x', word );
    end
end    
fprintf('\n');toc
fclose(fp);

end

function fdisp( out, format, varargin )
    global gvars;
    if( gvars.verbose == 1 )
        fprintf( out, format, varargin{:} );
    end
end
