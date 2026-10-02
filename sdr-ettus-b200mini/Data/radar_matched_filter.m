%% radar_matched_filter.m
% MATLAB port of the Python matched-filter / Range-Time-Intensity (RTI)
% processing script.
%
% Reads a reference TX chirp and a captured RX IQ file (both stored as
% interleaved int16 I/Q binary), performs pulse-by-pulse matched
% filtering (cross-correlation with the TX chirp), and plots:
%   1) the reference TX signal
%   2) a handful of raw RX pulses + their matched-filter output
%   3) the full Range-Time-Intensity (RTI) image
%
% All plots are saved as PNGs into a timestamped folder under plots/.

clear; clc; close all;

%% ======================= EDITABLE PARAMETERS ==========================

% --- File paths ---read_int16_iq
tx_file   = "Data/tx_chirp_only.bin";
json_file = "parameters/params.json";
rx_file   = "Data/rx_data_20260909_172015.bin"; % rx 0.5, same port
rx_file   = "Data/rx_data_20260909_173318.bin"; % rx 0.7, same port
rx_file = "Data/rx_data_20260909_174304.bin"; % rx 0.7 tx 1 f = 5.85, same port
rx_file = "Data/rx_data_20260909_174609.bin"; % rx 0.8 tx 1 f = 5.85, same port

rx_file = "Data/rx_data_20260909_174809.bin"; % rx 0.5 tx 1 f = 5.85, different port
rx_file = "Data/rx_data_20260909_174951.bin"; % rx 0.9 tx 1 f = 5.85, different port

rx_file = "Data/rx_data_20260909_175551.bin";% rx 0.9 tx 1 f = 5.85, different port, 2chirps
rx_file = "Data/rx_data_20260909_180416.bin";% rx 1 tx 1 f = 5.85, different port, 2chirps
rx_file = "Data/rx_data_20260909_180841.bin";% rx 1 tx 1 f = 5.85, same port, 2chirps

rx_file = "Data/rx_data_20260909_181515.bin";% rx 1 tx 1 f = 5.85, same port, 2chirps
rx_file = "Data/rx_data_20260909_181718.bin";% rx 1 tx 1 f = 5.85, different port, 2chirps

rx_file = "Data/rx_data_20260909_182221.bin";
% --- Pulses to inspect individually in the raw/compressed pulse plot ---
pulse_indices = [11, 12, 13, 14, 15, 15, 16, 18, 19, 20 100, 101, 501];   % MATLAB is 1-indexed
                                                 % (Python indices 10,11,12,99,100,500)

% --- Output folder for saved plots ---
plots_root = "plots";

% =========================================================================
% Everything below this line generally does not need to be edited.
% =========================================================================

%% --- Load config (PRF, FS, T_MIN_US, T_MAX_US) from JSON ---
config = jsondecode(fileread(json_file));

PRF      = config.PRF;
FS       = config.FS;
T_MIN_US = 0;
T_MAX_US = config.T_MAX_US-config.T_MIN_US;

window_samps = round((T_MAX_US - T_MIN_US) * 1e-6 * FS);

%% --- Load reference TX chirp ---
raw_tx = read_int16_iq(tx_file);
tx_chirp = raw_tx(1:2:end) + 1i * raw_tx(2:2:end);

%% --- Load RX data and reshape into pulses ---
raw_rx = read_int16_iq(rx_file);
IQ = raw_rx(1:2:end) + 1i * raw_rx(2:2:end);

[~, bin_name, ~] = fileparts(rx_file);

actual_pulses = floor(length(IQ) / window_samps);
valid_samples = actual_pulses * window_samps;
% reshape: MATLAB is column-major, so reshape then transpose to get
% data(pulse, sample) like the Python (actual_pulses, window_samps) array
data = reshape(IQ(1:valid_samples), window_samps, actual_pulses).';

%% --- Reference TX signal plot ---
tx_time_us = (0:length(tx_chirp)-1) / FS * 1e6;

fig_tx = figure();

subplot(2,1,1);
plot(tx_time_us, real(tx_chirp), 'Color', [0 0.4470 0.7410], ...
     'DisplayName', 'In-Phase (I)', 'LineWidth', 1.5); hold on;
plot(tx_time_us, imag(tx_chirp), 'Color', [0.8500 0.3250 0.0980], ...
     'DisplayName', 'Quadrature (Q)', 'LineWidth', 1.5);
title(sprintf('Original Reference Transmit Signal (%d samples)', length(tx_chirp)));
ylabel('Amplitude (int16 scale)');
legend('Location', 'northeast');
grid on;

subplot(2,1,2);
plot(tx_time_us, abs(tx_chirp), 'Color', [0.4660 0.6740 0.1880], ...
     'DisplayName', 'Magnitude', 'LineWidth', 1.5);
title('Reference Signal Envelope / Magnitude');
xlabel('Time [\mus]');
ylabel('Magnitude');
legend('Location', 'northeast');
grid on;

%% --- Pulse-by-pulse matched filtering & plotting ---
time_us = linspace(T_MIN_US, T_MAX_US, window_samps);

fig_rx = figure();
ax1 = subplot(2,1,1); hold(ax1, 'on');
ax2 = subplot(2,1,2); hold(ax2, 'on');

for k = 1:length(pulse_indices)
    idx = pulse_indices(k);
    rx_pulse = data(idx, :);

    compressed_pulse = matched_filter(rx_pulse, tx_chirp);

    plot(ax1, time_us, abs(rx_pulse), 'LineWidth', 1.5, ...
         'DisplayName', sprintf('Pulse %d', idx-1));
    plot(ax2, time_us, abs(compressed_pulse), 'LineWidth', 2, ...
         'DisplayName', sprintf('Pulse %d', idx-1));
end

title(ax1, 'Raw Received Pulse (Magnitude)');
ylabel(ax1, 'Amplitude (int16 scale)');
legend(ax1, 'Location', 'northeast');
grid(ax1, 'on');

title(ax2, 'Matched Filter Output (Cross-Correlation)');
xlabel(ax2, 'Time within RX Window [\mus]');
ylabel(ax2, 'Correlation Magnitude');
legend(ax2, 'Location', 'northeast');
grid(ax2, 'on');

%% --- Matched filtering across all pulses ---
fprintf('Applying matched filter to all %d pulses...\n', actual_pulses);
compressed_data = zeros(size(data));

for i = 1:actual_pulses
    compressed_data(i, :) = matched_filter(data(i, :), tx_chirp);
end

%% --- Range-Time Intensity (RTI) plot ---
session_duration = actual_pulses / PRF;

fig_rti = figure();
imagesc([T_MIN_US, T_MAX_US], [0, session_duration], abs(compressed_data));
set(gca, 'YDir', 'normal');
colormap(gca, 'parula');   % use 'viridis' colormap if you have it installed
colorbar_handle = colorbar;
ylabel(colorbar_handle, 'Correlation Magnitude');
xlabel('Time within RX Window [\mus] (Fast Time / Range)');
ylabel('Elapsed Time [s] (Slow Time)');
title(sprintf('Range-Time Intensity Image (%d pulses)', actual_pulses));
grid off;


%% Check energy in all pusles

n_check = actual_pulses;  % first 200 pulses
energy = zeros(n_check, 1);
for i = 1:n_check
    energy(i) = sum(abs(data(i, :)).^2);
end

figure;
plot(0:n_check-1, energy, '.-');  % 0-indexed to match your C++/Python pulse numbering
xlabel('Pulse index (0-indexed)');
ylabel('Energy');
title('Per-pulse energy — look for periodic dropouts');


%% --- Save all plots ---
% timestamp = datestr(now, 'yyyymmdd_HHMMSS');
% save_folder = fullfile(plots_root, sprintf('matched_filter_%s_%s', timestamp, bin_name));
% if ~exist(save_folder, 'dir')
%     mkdir(save_folder);
% end
% 
% exportgraphics(fig_tx,  fullfile(save_folder, 'tx_signal.png'));
% exportgraphics(fig_rx,  fullfile(save_folder, 'rx_pulses.png'));
% exportgraphics(fig_rti, fullfile(save_folder, 'matched_filter_output.png'));
% fprintf('All plots saved to folder: %s\n', save_folder);


%% ============================ LOCAL FUNCTIONS ==========================

function raw = read_int16_iq(filepath)
    % Read a raw interleaved int16 IQ binary file
    fid = fopen(filepath, 'rb');
    if fid == -1
        error('Could not open file: %s', filepath);
    end
    raw = fread(fid, Inf, 'int16=>double');
    fclose(fid);
end

function out = matched_filter(rx, template)
    % Cross-correlate rx against template and return an array the SAME
    % LENGTH as rx, where out(n) is the correlation value corresponding
    % to the template match STARTING at rx(n).
    %
    % Equivalent to scipy.signal.correlate(rx, template, mode='full'),
    % sliced starting at index (len(template)-1) [0-indexed], which is
    % exactly where a match starting at rx(1) appears in the full
    % correlation. This avoids the +len(template)/2 sample bias that
    % mode='same' style correlation would introduce.
    %
    % Implementation note: cross-correlation full mode is equivalent to
    % convolution of rx with the conjugate, time-reversed template.

    rx = rx(:).';
    template = template(:).';

    full = conv(rx, conj(fliplr(template)));  % length = len(rx)+len(template)-1

    start_idx = length(template);             % 1-indexed start of the slice
    out = full(start_idx : start_idx + length(rx) - 1);
end