function mer_online( hostname )
%% 
% Sample for connecting to GL 5 TCPIP data server and retrieving depth, MER and
% LFP waveforms
%
% NOTE: This script only handles data streaming when one uE Interface is
% used together with the Guideline 5 system. Please contact FHC, Inc. for 
% extension of the functionality to streaming from a second interface. 
%
% Parameters inside the script:
%  show_mer_waveforms   values: true/false  Toggle for showing MER waveforms
%  show_lfp_waveforms   values: true/false  Toggle for showing LFP waveforms
%
% FHC Inc
% Cosmin Serban, 2021
%%

%addpath('..\');
legacy_codes;
message_codes;

% Switches
show_mer_waveforms = true;  % Switch for showing/not showing MER (true/false) 
show_lfp_waveforms = true;  % Switch for showing/ not showing LFP (true/false)

%TCPIP Parameters
if(nargin < 1)
    %hostname = '192.168.0.2';  % Local IP of the GL5 ethernet connection
    hostname = '127.0.0.1';    % Local IP (loopback)
end

% TCP Port, set on the GL5 Control Panel, Data Settings tab/
port    = 2567;
retry_intvl = 2.5;   % Interval in seconds after which a reconnect is attempted


% Parameters
mer_sample_rate = 32000; % Default MER sample rate. 
lfs_sample_rate =  1000;  % Default LFP sample rate. 

stim_sample_rate = 42088;   % Stimulator's sample rate, nominal 42090. This may need to be slightly adjusted to match system's clock

last_mer_ts = 0;
last_length_mer = 0;
last_lfs_ts = 0;

last_lfp_ts = 0;
last_lfp_len = 0;

% NSAMP and NSAMP_LFP set the display refresh rate 
NSAMP = 8000; % Default MER sampling rate is 32 kHz, 0.25 s refresh rate. 
NSAMP_LFP = 500; % Default LFP sampling rate is 1 kHz, 0.5 s refresh rate

mer_data = zeros(NSAMP, 8);
mer_time = zeros(NSAMP, 8);
mer_cnt = 1;

lfp_data = zeros(NSAMP, 8);
lfp_time = zeros(NSAMP, 8);
lfp_cnt = 1;


% Initialization
% Number of channels to monitor. This is subsequently adjusted on the fly as the
% configuration changes. 
nch = 8; 

ch_list = 1:nch;    % Channel list (e.g. [3 5 8])

% Figure handles
global gvars;
gvars.hcha_mer = []; 
gvars.hcha_lfs = [];   % Array of handles to the channel axes
gvars.hchw_mer = []; 
gvars.hchw_lfs = [];   % Array of handles to the channel waveform line

stim_on = false;

% Calibration constants to transform MER and LFP samples to microvolts:
% These are received when the connection is established
v_cal_mer = zeros(2,8); % MER voltage calibration - two interfaces. 
offset_dc_mer = zeros(2,8); % MER voltage offset for DC coupled input - two interfaces. Not used, as normally MER is always AC-coupled.
offset_ac_mer = zeros(2,8); % MER voltage offset for AC coupled input - two interfaces. 
v_cal_lfs = zeros(2,8); %  LFP voltage calibration - two interfaces. 
offset_cal_lfs = zeros(2,8); %  LFP voltage offset - two interfaces. 
v_cal_aux = zeros(1,2); % voltage calibration - sync interface, not used here

% Setup figures
if(show_mer_waveforms)
    setup_figures_mer(nch, ch_list);
end

if(show_lfp_waveforms)
    setup_figures_lfp(nch, ch_list);
end

colordef('black');
channel = [];
channel(1).syncs=[]; % Initialize the timestamps member
scrsz = get(0, 'ScreenSize');   % for sizing channel displays
% Initialize figure displaying drive depth
figure(101); clf; axis off;
set(gcf,'Position', [200 scrsz(4)-300 200 150],'Name', 'Drive 1', 'NumberTitle', 'off','Visible','Off');
figure(102); clf; axis off;
set(gcf,'Position', [400 scrsz(4)-300 200 150],'Name', 'Drive 1', 'NumberTitle', 'off','Visible','Off');
fpos = get(gcf,'Position');

pnet('closeall');
con=pnet('tcpconnect',hostname,port);
if (con == -1)
    disp 'Can''t connect to server!';
    return;
end
disp 'Connected to server';
stat=pnet(con,'status');
pnet(con,'setreadtimeout',60);  % Wait 60 seconds before aborting the read operation


while (stat)
    try
        m_code=double(pnet(con,'read',1,'uint32','intel'));           % Message type
        if( isempty(m_code))
            continue;
        end
        % Handle Guideline 4000 LP+ legacy messages
        switch (m_code)            % Handle legacy GL4K messages: 
            case MSG_MEAN_FIRING_RATE
                m_channel=double(pnet(con,'read',1,'uint32','intel'));        % Message channel & unit
                m_length=double(pnet(con,'read',1,'uint32','intel'));         % Message length
                m_data=double(pnet(con,'read',m_length,'uint32','intel'));    % Read message data, 32-bit integer
                mfr(m_channel) = m_data(1);
                %fprintf('\nChannel %d, mean firing rate %d',m_channel,m_data(1));

            case MSG_RMS
                m_channel=double(pnet(con,'read',1,'uint32','intel'));        % Message channel & unit
                m_length=double(pnet(con,'read',1,'uint32','intel'));         % Message length
                m_data=double(pnet(con,'read',m_length,'single','intel'));    % Read message data, 32-bit integer
                %rms = typecast( uint32(m_data(1)), 'single' ); 
                %disp(sprintf('Channel %d, RMS = %3.2f...',m_channel, m_data(1)));
                
            case MSG_DRIVE_DEPTH
                m_side=double(pnet(con,'read',1,'uint32','intel'));        % Message channel & unit
                %m_unit=bitshift(m_channel,-16);                     % Upper word of the channel may contain the unit for certain messages
                %m_channel=bitand(m_channel,hex2dec('0000FFFF'));    % Lower word of the channel contains the channel
                m_length=double(pnet(con,'read',1,'uint32','intel'));         % Message length

                %disp(sprintf(' Drive %d ...',m_channel));
                m_data=double(pnet(con,'read',m_length,'uint32','intel'));    % Read message data, 32-bit integer
                fprintf('\nDrive %d depth %d ...',m_side,m_data(1));
                figure(100+m_side); clf; axis off;
                fpos = get(gcf,'Position');
                % set(gcf,'Position', [(m_side-1)*220+1 scrsz(4)-300 200 200], ...  % window position depends on chan. # & screen size
                %     'Name', ['Drive ' num2str(m_side)], 'NumberTitle', 'off');
                set(gcf,'Position', [fpos(1) fpos(2) 200 200], 'Visible','On');
                text(0.5,0.5,num2str(m_data(1)),'FontWeight','bold', 'FontSize', 32,...
                    'HorizontalAlignment', 'center', 'VerticalAlignment','middle');
                
            % Handle GL5 specific messages. 
            case SYNC
                gl5_msg_code = double(pnet(con,'read',1,'uint32','intel'));
                msg_len=double(pnet(con,'read',1,'uint32','intel'));         % Message length
                
                % Parse and display MER
                if (gl5_msg_code == ADS_DATA_MER  )
                    %num_msgs = num_msgs + 1;

                    msg_chbitmap = double(pnet(con,'read',1,'uint32','intel'));
                    msg_timestamp = double(pnet(con,'read',1,'uint32','intel'));
                    msg_ch_list = find(flip(dec2bin(bitand(msg_chbitmap,  hex2dec( '0FFFFFFF') )))-'0');   % Convert msg_chbitmap to array, then find indices of ones
                    num_channels = length(msg_ch_list);
                    num_samples = (msg_len-2)/ num_channels;
                    msg_seq = bitshift( bitand( msg_chbitmap, hex2dec( 'F0000000' )), -28);
                    msg_seq = msg_seq + 1;
                    
                    if(last_mer_ts > 0)
                        if( msg_timestamp - last_mer_ts > last_length_mer)
                            fprintf('\nmissing samples, ts = %d, last_ts = %d, expected ts = %d, d = %d', msg_timestamp, last_mer_ts, last_mer_ts + num_samples, msg_timestamp - last_mer_ts);
                        end
                    end
                    last_mer_ts = msg_timestamp;
                    last_length_mer = num_samples;
                    last_mer_time = last_mer_ts / mer_sample_rate;
                    channel_data = double(pnet(con,'read',msg_len-2,'int32','intel'));

                    % Setup figures whenever the number of channels has changed
                    if (show_mer_waveforms)
                        if (num_channels~=nch)
                            nch = num_channels;
                            ch_list = msg_ch_list;
                            setup_figures_mer(nch, ch_list);

                        elseif any(msg_ch_list~=ch_list)
                            setup_figures_mer(nch, ch_list);
                        end
                        channel_cntr = 1;
                        for i = 1:num_channels
                            data = channel_data(channel_cntr: channel_cntr -1 + num_samples);
                            channel_cntr = channel_cntr + num_samples;                                
                            mer_data(mer_cnt:mer_cnt + num_samples-1,i) = ( (v_cal_mer(msg_seq,ch_list(i)).* data) - offset_ac_mer(msg_seq,ch_list(i)) ).';                                

                            mer_time((mer_cnt):mer_cnt+num_samples-1, i) = mer_cnt/mer_sample_rate + (0:(num_samples-1)) / mer_sample_rate;
                        end
                        mer_cnt = mer_cnt + num_samples;

                        if(mer_cnt >= NSAMP)    
                            for i = 1:num_channels

                                % Display data for demo purposes:

                                gvars.hchw_mer(i) = plot(gvars.hcha_mer(i),mer_time(:,i), mer_data(:,i),'-','Color',[0.2 0.2 1]);
                                ylim(gvars.hcha_mer(i),max(abs([ylim.'; mer_data(:,i)]))*[-1 1]);
                                drawnow;

                            end
                            mer_data = zeros(NSAMP,8);
                            mer_time = zeros(NSAMP,8);
                            mer_cnt = 1;
                        end
                        
                    end
                % ---------------------------------------------------------
                elseif(gl5_msg_code == LFP_DATA_RAW)
  
                    msg_chbitmap = double(pnet(con,'read',1,'uint32','intel'));
                    msg_timestamp = double(pnet(con,'read',1,'uint32','intel'));
                    msg_ch_list = find(flip(dec2bin(bitand(msg_chbitmap,  hex2dec( '0FFFFFFF') )))-'0');   % Convert msg_chbitmap to array, then find indices of ones
                    num_channels = length(msg_ch_list);
                    num_samples = (msg_len-2)/ num_channels;
                    msg_seq = bitshift( bitand( msg_chbitmap, hex2dec( 'F0000000' )), -28);
                    msg_seq = msg_seq + 1;
                    
                    if( last_lfp_ts > 0 )
                        if( msg_timestamp - last_lfp_ts > double(( double(mer_sample_rate)/double(lfs_sample_rate))*last_lfp_len)) 
                              fprintf( '\nLFP data loss, expected: %d, found: %d, dif = %d', ...
                                  last_lfp_ts + 32 * last_lfp_len, msg_timestamp, (last_ts + 32 * last_lfp_len) - msg_timestamp );
                        else
                            %fprintf( '\nLFP timestamp OK: last = %d, new: %d', last_ts, msg_timestamp);
                        end
                        last_lfp_ts = msg_timestamp;
                        last_lfp_len = num_samples;
                    end                    
                                     
                    channel_data = double(pnet(con,'read',msg_len-2,'int32','intel'));
                    %if(msg_seq == 2)
                        % Setup figures whenever the number of channels has changed
                        if (show_lfp_waveforms)
                            if (num_channels~=nch)
                                nch = num_channels;
                                ch_list = msg_ch_list;
                                setup_figures_lfp(nch, ch_list);

                            elseif any(msg_ch_list~=ch_list)
                                setup_figures_lfp(nch, ch_list);
                            end
                            channel_cntr = 1;
                            for i = 1:num_channels
                                data = channel_data(channel_cntr: channel_cntr -1 + num_samples);
                                channel_cntr = channel_cntr + num_samples;                                
                                lfp_data(lfp_cnt:lfp_cnt + num_samples-1,i) = ( (v_cal_lfs(msg_seq,ch_list(i)).* data) - offset_cal_lfs(msg_seq,ch_list(i)) ).';                                

                                lfp_time(lfp_cnt:lfp_cnt+num_samples-1, i) = lfp_cnt/lfs_sample_rate + (0:(num_samples-1)) / lfs_sample_rate;
                            end
                            lfp_cnt = lfp_cnt+ num_samples;

                            if(lfp_cnt >= NSAMP_LFP)    
                                for i = 1:num_channels

                                    % Display data for demo purposes:

                                    gvars.hchw_lfs(i) = plot(gvars.hcha_lfs(i), lfp_time(:,i), lfp_data(:,i),'-','Color',[1 0.2 0.2]);
                                    ylim(gvars.hcha_lfs(i),max(abs([ylim.'; lfp_data(:,i)]))*[-1 1]);
                                    drawnow;

                                end
                                lfp_data = zeros(NSAMP_LFP,8);
                                lfp_time = zeros(NSAMP_LFP,8);
                                lfp_cnt = 1;
                            end
                        end
                    %end
  
                % ---------------------------------------------------------
                elseif(gl5_msg_code == HMS_EVENT_BOARD_TYPE)
                    m_board_type = double(pnet(con,'read',1,'uint32','intel'));
                    m_device_id = double(pnet(con,'read',1,'uint32','intel'));
                    fprintf( 1,'\nBoard type = %d, device id = %d', m_board_type, m_device_id);
                    
                % ---------------------------------------------------------
                elseif(gl5_msg_code == HMS_EVENT_MER_SAMPLING_RATE)
                    mer_sample_rate = double(pnet(con,'read',1,'uint32','intel'));
                    fprintf( 1,'\nMER sample rate = %d', mer_sample_rate);
                    
                % ---------------------------------------------------------
                elseif(gl5_msg_code == HMS_EVENT_LFS_SAMPLING_RATE)
                    lfs_sample_rate = double(pnet(con,'read',1,'uint32','intel'));
                    fprintf( 1,'\nLFS sample rate = %d', lfs_sample_rate);
                    
                % ---------------------------------------------------------
                elseif(gl5_msg_code == HMS_EVENT_MER_VOLTAGE_CALIBRATION)
                    m_channel = int32(pnet(con,'read',1,'uint32','intel'));
                    m_device_id = int32(pnet(con,'read',1,'uint32','intel'));
%                     m_vcal = uint32(pnet(con,'read',1,'uint32','intel'));
%                     m_vcal = typecast(m_vcal, 'single');
                    m_vcal = double(pnet(con,'read',1,'single','intel'));
                    m_ofscal_dc = double(pnet(con,'read',1,'int32','intel'));
                    m_ofscal_ac = double(pnet(con,'read',1,'int32','intel'));
                    fprintf(1, '\nMER voltage calibration on device %d, channel %d = %f, offset %d', ...
                        m_device_id, m_channel, m_vcal,m_ofscal_ac);
                    if(m_device_id >= 0 && m_device_id < size(v_cal_mer,1))
                        if(m_channel >= 0 && ( (m_channel) < size(v_cal_mer,2)))
                            v_cal_mer(m_device_id+1,m_channel+1) = m_vcal;
                            offset_dc_mer(m_device_id+1,m_channel+1) = m_ofscal_dc;
                            offset_ac_mer(m_device_id+1,m_channel+1) = m_ofscal_ac;
                        else
                            fprintf(1,'\nMER channel out of range %d', m_channel);
                        end
                    else
                        fprintf(1,'\nMER device id out of range %d', m_device_id);
                    end                    
                % ---------------------------------------------------------
                elseif(gl5_msg_code == HMS_EVENT_LFP_VOLTAGE_CALIBRATION)
                    m_channel = int32(pnet(con,'read',1,'uint32','intel'));
                    m_device_id = int32(pnet(con,'read',1,'uint32','intel'));
%                     m_vcal = uint32(pnet(con,'read',1,'uint32','intel'));
%                     m_vcal = typecast(m_vcal, 'single');
                    m_vcal = double(pnet(con,'read',1,'single','intel'));
                    m_ofscal = double(pnet(con,'read',1,'int32','intel'));
                    fprintf(1, '\nLFP voltage calibration on device %d, channel %d = %f, offset %d', ...
                        m_device_id, m_channel, m_vcal,m_ofscal);                    
                    if(m_device_id >= 0 && m_device_id < size(v_cal_lfs,1))
                        if(m_channel >= 0 && ( (m_channel) < size(v_cal_lfs,2)))
                            v_cal_lfs(m_device_id+1,m_channel+1) = m_vcal;
                            offset_cal_lfs(m_device_id+1,m_channel+1) = m_ofscal;
                        else
                            fprintf(1,'\nMER channel out of range %d', m_channel);
                        end
                    else
                        fprintf(1,'\nMER device id out of range %d', m_device_id);
                    end                    
                    
                elseif(gl5_msg_code == STIM_EVENT_STIM_TYPE)
                   m_data = double(pnet(con,'read',msg_len,'uint32','intel'));
                   device_id = m_data(1);
                   stim_type = m_data(2);
                   fprintf('\nDevice %d: stimulation type: 0x%x', device_id, stim_type);
                   
                elseif(gl5_msg_code == STIM_EVENT_STIM_ON)
                   m_data = double(pnet(con,'read',msg_len,'uint32','intel'));
                   device_id = m_data(1);
                   stim_on = true;
                   stim_on_ts = m_data(2);
                   fprintf('\n Last LFS TS: 0x%x, %.3f s', last_lfs_ts, last_lfs_ts / mer_sample_rate);
                   fprintf('\nDevice %d: stimulation on TS: 0x%x, %.3f s', device_id, stim_on_ts, stim_on_ts / mer_sample_rate);
                    
                elseif(gl5_msg_code == STIM_EVENT_STIM_OFF)
                   m_data = double(pnet(con,'read',msg_len,'uint32','intel'));
                   device_id = m_data(1);
                   stim_on = false;
                   stim_off_ts = m_data(2);
                   fprintf('\nDevice %d: stimulation on TS: 0x%x, %.3f s', device_id, stim_off_ts, stim_off_ts / mer_sample_rate);
                   
                elseif(gl5_msg_code == STIM_EVENT_PULSE_FREQUENCY)
                   m_data = double(pnet(con,'read',msg_len,'uint32','intel'));
                   device_id = m_data(1);
                   output_channel = m_data(2);
                   frequency = m_data(3);
                   fprintf('\nDevice %d: channel %d, stimulation frequency: %d', ...
                       device_id, output_channel, frequency);

                elseif(gl5_msg_code == STIM_EVENT_PULSE_AMPLITUDE)
                   m_data = double(pnet(con,'read',msg_len,'uint32','intel'));
                   device_id = m_data(1);
                   output_channel = m_data(2);
                   amplitude = m_data(3);
                   fprintf('\nDevice %d: channel %d, stimulation pulse amplitude: %d', ...
                       device_id, output_channel, typecast( amplitude, 'single'));

                elseif(gl5_msg_code == STIM_EVENT_PULSE_DURATION)
                   m_data = double(pnet(con,'read',msg_len,'uint32','intel'));
                   device_id = m_data(1);
                   output_channel = m_data(2);
                   duration = m_data(3);
                   fprintf('\nDevice %d: channel %d, stimulation pulse duration (ticks): %d', ...
                       device_id, output_channel, duration);

                elseif(gl5_msg_code == STIM_EVENT_PULSE_PHASE)
                   m_data = double(pnet(con,'read',msg_len,'uint32','intel'));
                   device_id = m_data(1);
                   output_channel = m_data(2);
                   phase = m_data(3);
                   fprintf('\nDevice %d: channel %d, stimulation pulse phase: %d', ...
                   device_id, output_channel, phase);
               elseif(gl5_msg_code == STIM_EVENT_PULSE_POLARITY)
                   m_data = double(pnet(con,'read',msg_len,'uint32','intel'));
                   device_id = m_data(1);
                   output_channel = m_data(2);
                   polarity = m_data(3);% 0 - negative, 1 - positive
                   fprintf( 1, '\nDevice %d: channel %d, stimulation pulse polarity: %d', ...
               device_id, output_channel, polarity);   
                elseif(gl5_msg_code == STIM_EVENT_OUTPUT_CHANNELS)
                   m_data = double(pnet(con,'read',msg_len,'uint32','intel'));
                   device_id = m_data(1);
                   output_channel = m_data(2);
                   channel_map = m_data(3);
                   fprintf('\nDevice %d: channel %d, stimulation output channels (hex map): 0x%x', ...
                       device_id, output_channel, channel_map);                

                elseif(gl5_msg_code == STIM_EVENT_RETURN_CHANNELS)
                   m_data = double(pnet(con,'read',msg_len,'uint32','intel'));
                   device_id = m_data(1);
                   output_channel = m_data(2);
                   channel_map = m_data(3);
                   fprintf('\nDevice %d: channel %d, stimulation return channels (hex map): 0x%x', ...
                       device_id, output_channel, channel_map);   
                elseif(gl5_msg_code==STIM_EVENT_OPERATING_MODE)
                   m_data = double(pnet(con,'read',msg_len,'uint32','intel'));
                   device_id = m_data(1);
                   stim_mode = m_data(2); % 0 - Constant Current, 1 - Constant Voltage
                   fprintf( 1, '\nDevice %d: stimulation type: 0x%x', device_id, stim_mode);      
   
                else
                   %fprintf('\n1. Unknown message code 0x%x',gl5_msg_code);
                   m_data = double(pnet(con,'read',msg_len,'uint32','intel'));
                end                

            otherwise
                fprintf('\n2. Unknown message code 0x%x',m_code);%, m_length);               
        end
    catch exc
        disp(exc);
        for i = 1:length(exc.stack)
            disp(exc.stack(i));
        end
        % Connection has been closed, retry connecting after retry_intvl seconds
        fprintf('Connection closed, reason: \n %s\n Retrying ...',lasterr);
        %disp('Connection closed, retrying ...');
        pause(retry_intvl);
        con=pnet('tcpconnect',hostname,port);
        if (con == -1)
            disp('Connection closed, retrying ...');
            pause(retry_intvl);
            con=pnet('tcpconnect',hostname,port);
        end
    end
    if (con ~= -1)    
        stat=pnet(con,'status');
    else
        stat = 0;   % This will exit the loop
    end
end
if (con ~= -1)    
    pnet(con,'close');
end

%close(1);

end

function setup_figures_mer(nch, ch_list)
    global gvars;

    % MER figure
    sz = get(0, 'Screensize');

    figure(1);clf;
    set(gcf,'Name','MER');
    set(gcf,'Color','k');
    fpos = get(gcf,'Position');
    fpos(1) = sz(3)* 0.2;
    fpos(4) = min(sz(4)-80,nch*120);
    fpos(2) = sz(4) - fpos(4)-80;
    set(gcf,'Position',fpos);

    for i = 1:nch
        gvars.hcha_mer(i) = subplot(nch,1,i);
        xlabel(sprintf('Time (ms)',ch_list(i)));
        ylabel(sprintf('Ch%d (\\muV)',ch_list(i)));
    end
end



function setup_figures_lfp(nch, ch_list)
    global gvars;

    sz = get(0, 'Screensize');

    % LFP figure
    figure(2);clf;
    set(gcf,'Name','LFS');
    set(gcf,'Color','k');
    fpos = get(gcf,'Position');
    fpos(1) = sz(3) * 0.6;
    fpos(4) = min(sz(4)-80,nch*120);
    fpos(2) = sz(4) - fpos(4) - 80;
    set(gcf,'Position',fpos);

    for i = 1:nch
        gvars.hcha_lfs(i) = subplot(nch,1,i);
        xlabel(sprintf('Time (ms)',ch_list(i)));
        ylabel(sprintf('Ch%d (\\muV)',ch_list(i)));
    end
end

