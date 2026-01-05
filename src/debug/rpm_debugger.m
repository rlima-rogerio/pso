% PSO Data Acquisition - RPM Debugger
% ==================================================================


clear all;
close all;
clc;

% load daq_period_us
load daq_period_ticks_75
% load daq_period_us_75

TICKS_MODE = true;
SAMPLING_RATE = 500;
BLADE_NUMBER = 2;
MIN_EDGE_PERIOD_US = 50;
RPM_FILTER_SAMPLES = 4;

% Global variables
global rpm_filter_buffer; 
global rpm_filter_index;
global rpm_filter_count;
global RPM_FILTER_SAMPLES;
global current_rpm;

% Global variables initialization
RPM_FILTER_SAMPLES = 4;
rpm_filter_buffer = zeros(1,RPM_FILTER_SAMPLES);
rpm_filter_index = 1;
rpm_filter_count = 0;

if (TICKS_MODE)
    period_ticks = data.rpm;
    period_us = (period_ticks + 20)/40;
else
    period_us = data.rpm;
end


t = data.index * (1/SAMPLING_RATE);
numSamples = length(t);

rpm = 60000000 ./ (period_us * BLADE_NUMBER);
rpm_filt = zeros(1, numSamples);


% state = 'init';
% idx = 1; 
% last_period_us = period_us(1);
% count = 1;
% while idx < numSamples
%     
%     switch state
%         case 'init'
%             first_period_us = period_us(idx);
% %             idx = idx + 1;
%             current_period_us = period_us(idx);
%             state = 'check_feasible_period';
%             
%         case 'check_feasible_period'
%             if (current_period_us < MIN_EDGE_PERIOD_US) || (last_period_us == current_period_us)
%                 idx = idx + 1;
%                 last_period_us = current_period_us;
%                 current_period_us = period_us(idx);
%                 state = 'check_feasible_period';
%             else
%                 state = 'valid_period';
%             end
%             
%         case 'valid_period'
%             valid_period_us = current_period_us - first_period_us;
%             rpm(count) = 60000000 ./ (valid_period_us * BLADE_NUMBER);
%             count = count + 1;
%             idx = idx + 1;
%             last_period_us = current_period_us;
%             current_period_us = period_us(idx);
%             state = 'init';          
%     end
%         
% end

for k = 1 : numSamples
    current_rpm = rpm(k);
    rpm_update_filter(current_rpm);
    rpm_filt(k) = rpm_get_filtered();
end


figure,
subplot(3,1,1), plot(t, data.throttle, 'g-'); hold on;
subplot(3,1,2), plot(t, period_us, 'r-'); 
subplot(3,1,3), plot(t, rpm_filt, 'g-');
grid on;


function rpm_update_filter(new_rpm)
    global rpm_filter_buffer;
    global rpm_filter_index;
    global rpm_filter_count;
    global RPM_FILTER_SAMPLES;
    
    rpm_filter_buffer(rpm_filter_index) = new_rpm;
    rpm_filter_index = mod((rpm_filter_index + 1), RPM_FILTER_SAMPLES) + 1;
    
    if (rpm_filter_count < RPM_FILTER_SAMPLES)
    
        rpm_filter_count = rpm_filter_count + 1;
    end
end

function g_rpm_value = rpm_get_filtered()
    global rpm_filter_buffer;
    global rpm_filter_count;
    global current_rpm;
    
    sum = 0;
    
    if (rpm_filter_count == 1)
        g_rpm_value = current_rpm;
        return;
    end
    
    for i = 1 : rpm_filter_count
        sum = sum + rpm_filter_buffer(i);
    end
    
    g_rpm_value = sum / rpm_filter_count;
end