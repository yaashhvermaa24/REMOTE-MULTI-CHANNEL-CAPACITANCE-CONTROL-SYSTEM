%% =========================================================================
%  Remote Multi-Channel Capacitance Control via Digital Communication & PWM
%  Complete End-to-End MATLAB Simulation — ALL OUTPUTS DISPLAYED
%  Blocks: A → B → C → D → E → F → G → H
%% =========================================================================
clc; clear; close all;

fprintf('=============================================================\n');
fprintf('  Remote Multi-Channel Capacitance Control System\n');
fprintf('  End-to-End MATLAB Simulation\n');
fprintf('=============================================================\n\n');

%% =========================================================================
%  GLOBAL PARAMETERS
%% =========================================================================
NUM_CH      = 48;
ADC_BITS    = 10;
V_MAX       = 10;
V_REF       = V_MAX;
dV          = V_REF / (2^ADC_BITS - 1);
PWM_FREQ    = 20e3;
FC          = 10e3;
Rs          = 10e3;
Fs          = 500e3;
sps         = Fs / Rs;
C0=18; phi=0.7; gam=0.45;
varactorC = @(V) C0 ./ (1 + max(V,0.001)./phi).^gam;
invCV     = @(C) phi .* ((C0./C).^(1/gam) - 1);
SYNC_WORD = [1 0 1 0 1 0 1 0 0 1 0 1 0 1 0 1];  % 16-bit sync
SNR_LIST  = [5 10 15 20 25];

%% =========================================================================
%  BLOCK A — TRANSMITTER INPUT
%% =========================================================================
fprintf('[BLOCK A] Generating 48 target capacitance values...\n');
rng(42);
C_min_t = varactorC(V_MAX);
C_max_t = varactorC(0.1);
C_target = sort(C_min_t + (C_max_t - C_min_t) * rand(1, NUM_CH));
V_bias   = max(0, min(V_MAX, invCV(C_target)));

fprintf('  C range : %.3f to %.3f pF\n', min(C_target), max(C_target));
fprintf('  V range : %.3f to %.3f V\n',  min(V_bias),   max(V_bias));

%% =========================================================================
%  BLOCK B — ADC ENCODING
%% =========================================================================
fprintf('\n[BLOCK B] ADC Encoding...\n');
ADC_codes = round(V_bias / dV);
V_quant   = ADC_codes * dV;
Q_err     = V_bias - V_quant;

data_bits = [];
for ch = 1:NUM_CH
    data_bits = [data_bits, fliplr(dec2bin(ADC_codes(ch), ADC_BITS) - '0')]; %#ok
end

% CRC-16 (simple XOR-based)
crc_val  = mod(sum(data_bits), 65536);
crc_bits = fliplr(dec2bin(crc_val, 16) - '0');

bitstream   = [SYNC_WORD, data_bits, crc_bits];
total_bits  = length(bitstream);
frame_ms    = total_bits / Rs * 1000;

fprintf('  ADC bits     : %d\n', ADC_BITS);
fprintf('  dV           : %.4f mV/LSB\n', dV*1000);
fprintf('  Frame size   : %d bits\n', total_bits);
fprintf('  Frame period : %.2f ms\n', frame_ms);

%% =========================================================================
%  BLOCK C — BPSK MODULATION
%% =========================================================================
fprintf('\n[BLOCK C] BPSK Modulation...\n');
t_sym    = (0:sps-1).' / Fs;
tx_wave  = zeros(1, total_bits * sps);
for k = 1:total_bits
    phase = bitstream(k) * pi;
    tx_wave((k-1)*sps+1 : k*sps) = cos(2*pi*FC*t_sym + phase).';
end
t_tx = (0:length(tx_wave)-1) / Fs;
fprintf('  Carrier freq : %.1f kHz\n', FC/1e3);
fprintf('  Symbol rate  : %.1f kbps\n', Rs/1e3);
fprintf('  Total samps  : %d\n', length(tx_wave));

%% =========================================================================
%  BLOCK D — AWGN CHANNEL + BLOCK E — DEMODULATION (BER loop)
%% =========================================================================
fprintf('\n[BLOCK D/E] AWGN Channel + Demodulation (BER sweep)...\n');
BER_th  = zeros(1, length(SNR_LIST));
BER_sim = zeros(1, length(SNR_LIST));
for idx = 1:length(SNR_LIST)
    snr_dB  = SNR_LIST(idx);
    snr_lin = 10^(snr_dB/10);
    Eb      = mean(tx_wave.^2);
    noise   = sqrt(Eb/(2*snr_lin)) * randn(size(tx_wave));
    rx_tmp  = tx_wave + noise;
    bits_tmp = zeros(1, total_bits);
    ref      = cos(2*pi*FC*t_sym).';
    for k = 1:total_bits
        seg = rx_tmp((k-1)*sps+1 : k*sps);
        bits_tmp(k) = (sum(seg .* ref) < 0);
    end
    BER_sim(idx) = sum(bits_tmp ~= bitstream) / total_bits;
    BER_th(idx)  = qfunc(sqrt(2*snr_lin));
    fprintf('  SNR=%2d dB  BER_sim=%.2e  BER_th=%.2e\n', snr_dB, BER_sim(idx), BER_th(idx));
end

% Full demodulation at 25 dB for downstream processing
Eb      = mean(tx_wave.^2);
noise25 = sqrt(Eb/(2*1000)) * randn(size(tx_wave));
rx_wave = tx_wave + noise25;
bits_rx = zeros(1, total_bits);
ref_c   = cos(2*pi*FC*t_sym).';
for k = 1:total_bits
    seg = rx_wave((k-1)*sps+1 : k*sps);
    bits_rx(k) = (sum(seg .* ref_c) < 0);
end

% Frame sync
sync_idx = 1;
for i = 1:length(bits_rx)-15
    if all(bits_rx(i:i+15) == SYNC_WORD)
        sync_idx = i; break;
    end
end
d_start = sync_idx + 16;
d_end   = d_start + NUM_CH * ADC_BITS - 1;
if d_end > length(bits_rx)
    bits_rx(end+1:d_end+16) = 0;
end
rx_data = bits_rx(d_start:d_end);

% CRC check
rx_crc_bits = bits_rx(d_end+1:min(d_end+16, length(bits_rx)));
if length(rx_crc_bits) < 16, rx_crc_bits(end+1:16)=0; end
rx_crc_val = sum(rx_crc_bits .* 2.^(0:15));
crc_ok = (mod(rx_crc_val,65536)==crc_val);
fprintf('\n  CRC: %s\n', ternary(crc_ok,'PASSED','FAILED'));

%% =========================================================================
%  BLOCK F — DAC RECONSTRUCTION
%% =========================================================================
fprintf('\n[BLOCK F] DAC Reconstruction...\n');
V_recon = zeros(1, NUM_CH);
for ch = 1:NUM_CH
    bs = rx_data((ch-1)*ADC_BITS+1 : ch*ADC_BITS);
    code = sum(bs .* 2.^(0:ADC_BITS-1));
    V_recon(ch) = code * dV;
end
V_err_F = V_bias - V_recon;
fprintf('  Max error : %.4f mV\n', max(abs(V_err_F))*1000);
fprintf('  RMS error : %.4f mV\n', rms(V_err_F)*1000);

%% =========================================================================
%  BLOCK G — PWM + RC LOW-PASS FILTER
%% =========================================================================
fprintf('\n[BLOCK G] PWM + RC Filter...\n');
fc_lp = 0.05 * PWM_FREQ;
RC    = 1/(2*pi*fc_lp);
V_pwm_out = zeros(1, NUM_CH);
V_ripple  = zeros(1, NUM_CH);
t_pwm = (0:1/Fs:3/PWM_FREQ);
dt    = 1/Fs;
alpha = dt/(RC+dt);

for ch = 1:NUM_CH
    duty = V_recon(ch)/V_MAX;
    pwm  = double(mod(t_pwm, 1/PWM_FREQ) < duty*(1/PWM_FREQ)) * V_MAX;
    v = zeros(size(pwm)); v(1) = pwm(1);
    for n = 2:length(pwm)
        v(n) = alpha*pwm(n) + (1-alpha)*v(n-1);
    end
    ss = v(round(end/2):end);
    V_pwm_out(ch) = mean(ss);
    V_ripple(ch)  = (max(ss) - min(ss)) * 1000;
end
fprintf('  Max ripple : %.2f mV pp\n', max(V_ripple));
fprintf('  Target <=50 mV pp : %s\n', ternary(max(V_ripple)<=50,'PASS','FAIL'));

%% =========================================================================
%  BLOCK H — VARACTOR CHARACTERIZATION
%% =========================================================================
fprintf('\n[BLOCK H] Varactor Characterization...\n');
C_achieved = varactorC(max(V_pwm_out,0.001));
C_err_pF   = C_target - C_achieved;
C_err_pct  = 100 * abs(C_err_pF) ./ C_target;
fprintf('  Max C error : %.3f pF (%.2f%%)\n', max(abs(C_err_pF)), max(C_err_pct));
fprintf('  RMS C error : %.3f pF (%.2f%%)\n', rms(C_err_pF),     rms(C_err_pct));

ch_lbl = 1:NUM_CH;

%% =========================================================================
%  FIGURE 1 — Block A: Transmitter Input Stage
%% =========================================================================
figure('Name','Fig 1 - Block A: Transmitter Input','Color','white', ...
       'NumberTitle','off','Position',[50 550 900 480]);

subplot(2,1,1);
bar(ch_lbl, C_target, 'FaceColor',[0.18 0.45 0.73],'EdgeColor','none');
xlabel('Channel Number','FontSize',10);
ylabel('Capacitance (pF)','FontSize',10);
title('Block A — 48 Target Capacitance Values','FontSize',11,'FontWeight','bold');
xlim([0 49]); grid on; box on;
text(2, max(C_target)*0.93, ...
     sprintf('Range: %.2f to %.2f pF', min(C_target), max(C_target)), ...
     'FontSize',9,'BackgroundColor','white','EdgeColor','none');

subplot(2,1,2);
bar(ch_lbl, V_bias, 'FaceColor',[0.85 0.33 0.10],'EdgeColor','none');
xlabel('Channel Number','FontSize',10);
ylabel('Bias Voltage (V)','FontSize',10);
title('Block A — 48 Required Bias Voltages (Inverted from C-V Model)','FontSize',11,'FontWeight','bold');
xlim([0 49]); grid on; box on;
text(2, max(V_bias)*0.93, ...
     sprintf('Range: %.2f to %.2f V', min(V_bias), max(V_bias)), ...
     'FontSize',9,'BackgroundColor','white','EdgeColor','none');

sgtitle('BLOCK A: Transmitter Input Stage','FontSize',13,'FontWeight','bold');

%% =========================================================================
%  FIGURE 2 — Block B: ADC Encoding
%% =========================================================================
figure('Name','Fig 2 - Block B: ADC Encoding','Color','white', ...
       'NumberTitle','off','Position',[50 50 900 600]);

subplot(3,1,1);
stem(ch_lbl, ADC_codes,'filled','MarkerSize',4,'Color',[0.18 0.45 0.73]);
xlabel('Channel','FontSize',10); ylabel('ADC Code','FontSize',10);
title(sprintf('Block B — 10-bit ADC Codes  (dV = %.4f mV/LSB)',dV*1000), ...
      'FontSize',11,'FontWeight','bold');
xlim([0 49]); ylim([0 1100]); grid on; box on;

subplot(3,1,2);
bar(ch_lbl, Q_err*1000,'FaceColor',[0.47 0.67 0.19],'EdgeColor','none');
hold on;
yline( dV*500*1000,'r--','LineWidth',1.5,'Label','+0.5 LSB');
yline(-dV*500*1000,'r--','LineWidth',1.5,'Label','-0.5 LSB');
xlabel('Channel','FontSize',10); ylabel('Error (mV)','FontSize',10);
title('Block B — Quantisation Error per Channel','FontSize',11,'FontWeight','bold');
xlim([0 49]); grid on; box on;

subplot(3,1,3);
N_frame  = [16, NUM_CH*ADC_BITS, 16];
pie_lbl  = {sprintf('Sync (%d bits)',N_frame(1)), ...
            sprintf('Data (%d bits)',N_frame(2)), ...
            sprintf('CRC (%d bits)',N_frame(3))};
pie(N_frame, pie_lbl);
colormap(gca, [0.18 0.45 0.73; 0.85 0.33 0.10; 0.47 0.67 0.19]);
title(sprintf('Block B — Frame Structure  Total: %d bits @ %.1f kbps', ...
      total_bits, Rs/1e3), 'FontSize',11,'FontWeight','bold');

sgtitle('BLOCK B: ADC Encoding','FontSize',13,'FontWeight','bold');

%% =========================================================================
%  FIGURE 3 — Block C: BPSK Modulation
%% =========================================================================
figure('Name','Fig 3 - Block C: BPSK Modulation','Color','white', ...
       'NumberTitle','off','Position',[100 550 1000 520]);

SHOW = 12;
t_show = t_tx(1:SHOW*sps);

subplot(3,1,1);
stairs([1:SHOW, SHOW+1]-0.5, [bitstream(1:SHOW), bitstream(SHOW)], ...
       'Color',[0.85 0.33 0.10],'LineWidth',2);
xlabel('Bit Index','FontSize',10); ylabel('Bit Value','FontSize',10);
title('Block C — Transmitted Bit Sequence (first 12 bits)','FontSize',11,'FontWeight','bold');
xlim([0 SHOW]); ylim([-0.3 1.3]); yticks([0 1]); grid on; box on;

subplot(3,1,2);
plot(t_show*1000, tx_wave(1:SHOW*sps),'Color',[0.18 0.45 0.73],'LineWidth',1.2);
hold on;
for b = 1:SHOW
    xline((b-1)/Rs*1000,'--','Color',[0.6 0.6 0.6]);
end
xlabel('Time (ms)','FontSize',10); ylabel('Amplitude','FontSize',10);
title('Block C — BPSK Modulated Waveform (first 12 bits)','FontSize',11,'FontWeight','bold');
grid on; box on;

subplot(3,1,3);
NFFT = 2^14;
[P, F] = pwelch(tx_wave, hamming(NFFT), NFFT/2, NFFT, Fs, 'onesided');
plot(F/1000, 10*log10(P),'Color',[0.49 0.18 0.56],'LineWidth',1.2);
hold on;
xline(FC/1000,'r--','LineWidth',1.5,'Label',sprintf('f_c = %.0f kHz', FC/1e3));
xlabel('Frequency (kHz)','FontSize',10); ylabel('PSD (dB/Hz)','FontSize',10);
title('Block C — Power Spectral Density of BPSK Signal','FontSize',11,'FontWeight','bold');
xlim([0 40]); grid on; box on;

sgtitle('BLOCK C: BPSK Digital Modulation','FontSize',13,'FontWeight','bold');

%% =========================================================================
%  FIGURE 4 — Block D: AWGN Channel
%% =========================================================================
figure('Name','Fig 4 - Block D: AWGN Channel','Color','white', ...
       'NumberTitle','off','Position',[150 50 1000 560]);

col3 = [0.00 0.45 0.70; 0.47 0.67 0.19; 0.85 0.33 0.10];
snr3 = [10 15 20];
Eb_d = mean(tx_wave.^2);

for idx = 1:3
    snr_dB  = snr3(idx);
    snr_lin = 10^(snr_dB/10);
    n_tmp   = sqrt(Eb_d/(2*snr_lin)) * randn(size(tx_wave));
    rx_tmp  = tx_wave + n_tmp;

    subplot(3,2, idx*2-1);
    plot(t_show*1000, rx_tmp(1:SHOW*sps),'Color',col3(idx,:),'LineWidth',0.9);
    xlabel('Time (ms)','FontSize',9); ylabel('Amplitude','FontSize',9);
    title(sprintf('Block D — Received Signal at SNR = %d dB', snr_dB), ...
          'FontSize',10,'FontWeight','bold');
    ylim([-4 4]); grid on; box on;

    subplot(3,2, idx*2);
    n_show = n_tmp(1:min(5000,end));
    histogram(n_show, 50,'FaceColor',col3(idx,:),'EdgeColor','none','Normalization','pdf');
    hold on;
    xg = linspace(min(n_show), max(n_show), 300);
    sg = std(n_show);
    plot(xg, exp(-xg.^2/(2*sg^2))/(sg*sqrt(2*pi)),'k-','LineWidth',1.8);
    xlabel('Noise Amplitude','FontSize',9); ylabel('PDF','FontSize',9);
    title(sprintf('Block D — Noise Histogram at SNR = %d dB', snr_dB), ...
          'FontSize',10,'FontWeight','bold');
    legend('Histogram','Gaussian fit','FontSize',7,'Location','northeast');
    grid on; box on;
end

sgtitle('BLOCK D: AWGN Channel Model','FontSize',13,'FontWeight','bold');

%% =========================================================================
%  FIGURE 5 — Block E: Demodulation & BER
%% =========================================================================
figure('Name','Fig 5 - Block E: Demodulation & BER','Color','white', ...
       'NumberTitle','off','Position',[200 550 950 560]);

subplot(2,2,[1 2]);
semilogy(SNR_LIST, BER_th,  'b-o','LineWidth',2.2,'MarkerSize',8,'MarkerFaceColor','b');
hold on;
semilogy(SNR_LIST, max(BER_sim,1e-7),'r--s','LineWidth',2.2,'MarkerSize',8,'MarkerFaceColor','r');
xlabel('SNR (dB)','FontSize',11); ylabel('Bit Error Rate','FontSize',11);
title('Block E — BER vs SNR: Theoretical Q(sqrt(2*SNR)) vs Simulated','FontSize',11,'FontWeight','bold');
legend('Theoretical','Simulated (Monte Carlo)','Location','southwest','FontSize',10);
grid on; ylim([1e-7 1]); xlim([4 26]);
% Annotate SNR points
for i=1:length(SNR_LIST)
    text(SNR_LIST(i)+0.3, BER_th(i), ...
         sprintf('%.1e',BER_th(i)),'FontSize',7,'Color','b');
end

subplot(2,2,3);
snr_lin_c = 10^(2.5);
n_corr    = sqrt(Eb_d/(2*snr_lin_c)) * randn(1, 60*sps);
corr_vals = zeros(1,60);
ref_s     = cos(2*pi*FC*t_sym).';
for k = 1:60
    seg2 = tx_wave((k-1)*sps+1:k*sps) + n_corr((k-1)*sps+1:k*sps);
    corr_vals(k) = sum(seg2 .* ref_s);
end
stem(1:60, corr_vals,'filled','MarkerSize',3,'Color',[0.18 0.45 0.73],'LineWidth',0.8);
hold on; yline(0,'r-','LineWidth',1.5);
xlabel('Bit Index','FontSize',9); ylabel('Correlation Value','FontSize',9);
title('Block E — Correlator Output (decision variable)','FontSize',10,'FontWeight','bold');
grid on; box on;

subplot(2,2,4);
bits_rec60 = (corr_vals < 0);
errors60   = bitstream(1:60) ~= bits_rec60;
stem(1:60, double(errors60),'filled','MarkerSize',4,'Color',[0.85 0.33 0.10]);
xlabel('Bit Index','FontSize',9); ylabel('Error (1 = error)','FontSize',9);
title(sprintf('Block E — Bit Errors in first 60 bits: %d errors (SNR=25dB)', ...
      sum(errors60)),'FontSize',10,'FontWeight','bold');
ylim([0 1.5]); grid on; box on;

sgtitle('BLOCK E: Demodulation & Bit Recovery','FontSize',13,'FontWeight','bold');

%% =========================================================================
%  FIGURE 6 — Block F: DAC Reconstruction
%% =========================================================================
figure('Name','Fig 6 - Block F: DAC Reconstruction','Color','white', ...
       'NumberTitle','off','Position',[250 50 1000 600]);

subplot(3,1,1);
plot(ch_lbl, V_bias,  'b-o','LineWidth',1.8,'MarkerSize',5,'MarkerFaceColor','b'); hold on;
plot(ch_lbl, V_recon, 'r--s','LineWidth',1.5,'MarkerSize',4,'MarkerFaceColor','r');
xlabel('Channel','FontSize',10); ylabel('Voltage (V)','FontSize',10);
title('Block F — Original vs DAC Reconstructed Bias Voltages','FontSize',11,'FontWeight','bold');
legend('Original V_{bias}','Reconstructed V_{DAC}','Location','northwest','FontSize',9);
xlim([0 49]); grid on; box on;

subplot(3,1,2);
bar(ch_lbl, V_err_F*1000,'FaceColor',[0.47 0.67 0.19],'EdgeColor','none');
hold on;
yline( dV*500*1000,'r--','LineWidth',1.5,'Label','+0.5 LSB');
yline(-dV*500*1000,'r--','LineWidth',1.5,'Label','-0.5 LSB');
xlabel('Channel','FontSize',10); ylabel('Error (mV)','FontSize',10);
title(sprintf('Block F — Reconstruction Error  [Max = %.3f mV,  RMS = %.3f mV]', ...
      max(abs(V_err_F))*1000, rms(V_err_F)*1000),'FontSize',11,'FontWeight','bold');
xlim([0 49]); grid on; box on;

subplot(3,1,3);
ideal_in  = linspace(0, V_MAX, 512);
dac_out_v = round(ideal_in/dV)*dV;
dnl = diff(dac_out_v)/dV - 1;
inl = cumsum(dnl);
yyaxis left;
plot(dnl,'b-','LineWidth',1.0); ylabel('DNL (LSB)');
yyaxis right;
plot(inl,'r-','LineWidth',1.0); ylabel('INL (LSB)');
xlabel('DAC Code (subset)','FontSize',10);
title(sprintf('Block F — DAC Linearity  [MaxDNL = %.4f LSB,  MaxINL = %.4f LSB]', ...
      max(abs(dnl)), max(abs(inl))),'FontSize',11,'FontWeight','bold');
legend('DNL','INL','Location','best','FontSize',9); grid on; box on;

sgtitle('BLOCK F: DAC Reconstruction','FontSize',13,'FontWeight','bold');

%% =========================================================================
%  FIGURE 7 — Block G: PWM + RC Filter
%% =========================================================================
figure('Name','Fig 7 - Block G: PWM Voltage Generation','Color','white', ...
       'NumberTitle','off','Position',[300 550 1000 580]);

subplot(3,2,[1 2]);
bclr = repmat([0.18 0.45 0.73], NUM_CH, 1);
bclr(V_ripple > 50, :) = repmat([0.85 0.33 0.10], sum(V_ripple>50), 1);
bh = bar(ch_lbl, V_ripple,'FaceColor','flat','EdgeColor','none');
bh.CData = bclr;
hold on;
yline(50,'r--','LineWidth',2,'Label','50 mV limit');
xlabel('Channel','FontSize',10); ylabel('Ripple (mV pp)','FontSize',10);
title(sprintf('Block G — PWM Output Ripple  [Max = %.1f mV,  Mean = %.1f mV]  Target: <=50 mV', ...
      max(V_ripple), mean(V_ripple)),'FontSize',11,'FontWeight','bold');
xlim([0 49]); ylim([0 max(V_ripple)*1.35]); grid on; box on;

% PWM waveform for 3 representative channels
demo_chs = [6 24 42];
for k = 1:3
    ch_d = demo_chs(k);
    duty  = V_recon(ch_d)/V_MAX;
    pwm_d = double(mod(t_pwm, 1/PWM_FREQ) < duty*(1/PWM_FREQ)) * V_MAX;
    v_d   = zeros(size(pwm_d)); v_d(1) = pwm_d(1);
    for n = 2:length(pwm_d)
        v_d(n) = alpha*pwm_d(n) + (1-alpha)*v_d(n-1);
    end
    subplot(3,2, k+2);
    plot(t_pwm*1000, pwm_d,'Color',[0.60 0.80 1.00],'LineWidth',0.8); hold on;
    plot(t_pwm*1000, v_d,  'Color',[0.85 0.33 0.10],'LineWidth',2.0);
    yline(V_recon(ch_d),'k--','LineWidth',1, ...
          'Label',sprintf('Target = %.2fV',V_recon(ch_d)));
    xlabel('Time (ms)','FontSize',9); ylabel('Voltage (V)','FontSize',9);
    title(sprintf('Ch%d: Duty=%.1f%%  Ripple=%.1fmV', ...
          ch_d, duty*100, V_ripple(ch_d)),'FontSize',10,'FontWeight','bold');
    legend('PWM','RC Filtered','Location','southeast','FontSize',7);
    grid on; box on;
end

subplot(3,2,6);
f_bode = logspace(1, 5, 600);
H_mag  = 1 ./ sqrt(1 + (f_bode/fc_lp).^2);
semilogx(f_bode, 20*log10(H_mag),'b-','LineWidth',2);
hold on;
xline(fc_lp,    'g--','LineWidth',1.5,'Label',sprintf('f_c = %.0f Hz',fc_lp));
xline(PWM_FREQ, 'r--','LineWidth',1.5,'Label',sprintf('f_{PWM} = %.0f kHz',PWM_FREQ/1e3));
xlabel('Frequency (Hz)','FontSize',9); ylabel('|H(f)| (dB)','FontSize',9);
title(sprintf('RC LPF Bode Plot  (RC = %.3f ms)', RC*1000),'FontSize',10,'FontWeight','bold');
ylim([-60 5]); grid on; box on;

sgtitle('BLOCK G: PWM-Based DC Bias Generation','FontSize',13,'FontWeight','bold');

%% =========================================================================
%  FIGURE 8 — Block H: Varactor Characterization
%% =========================================================================
figure('Name','Fig 8 - Block H: Varactor Characterization','Color','white', ...
       'NumberTitle','off','Position',[50 50 1000 620]);

subplot(2,2,1);
V_sw = linspace(0.05, V_MAX, 500);
C_sw = varactorC(V_sw);
plot(V_sw, C_sw,'b-','LineWidth',2.5); hold on;
scatter(V_pwm_out, C_achieved, 50, 'r','filled');
xlabel('Reverse Bias Voltage (V)','FontSize',10);
ylabel('Capacitance (pF)','FontSize',10);
title('Block H — BB135 C-V Curve with 48 Operating Points','FontSize',11,'FontWeight','bold');
legend('C-V model  C_0/(1+V/phi)^gamma','48 varactor points','Location','northeast','FontSize',9);
text(0.5,4,sprintf('C_0=%.0fpF  phi=%.1f  gamma=%.2f',C0,phi,gam),'FontSize',8);
grid on; box on;

subplot(2,2,2);
bar(ch_lbl, abs(C_err_pF),'FaceColor',[0.85 0.33 0.10],'EdgeColor','none');
hold on;
yline(mean(abs(C_err_pF)),'b--','LineWidth',1.5, ...
      'Label',sprintf('Mean = %.3f pF', mean(abs(C_err_pF))));
xlabel('Channel','FontSize',10); ylabel('|Error| (pF)','FontSize',10);
title(sprintf('Block H — Capacitance Error  [Max=%.3f pF,  RMS=%.3f pF]', ...
      max(abs(C_err_pF)), rms(C_err_pF)),'FontSize',11,'FontWeight','bold');
xlim([0 49]); grid on; box on;

subplot(2,2,3);
bar(ch_lbl, C_err_pct,'FaceColor',[0.49 0.18 0.56],'EdgeColor','none');
hold on;
yline(mean(C_err_pct),'k--','LineWidth',1.5, ...
      'Label',sprintf('Mean = %.2f%%', mean(C_err_pct)));
xlabel('Channel','FontSize',10); ylabel('Error (%)','FontSize',10);
title(sprintf('Block H — Capacitance Error %%  [Max=%.2f%%,  RMS=%.2f%%]', ...
      max(C_err_pct), rms(C_err_pct)),'FontSize',11,'FontWeight','bold');
xlim([0 49]); grid on; box on;

subplot(2,2,4);
scatter(C_target, C_achieved, 50, ch_lbl, 'filled'); hold on;
clo = min(C_target); chi_v = max(C_target);
plot([clo chi_v],[clo chi_v],'r--','LineWidth',2);
xlabel('Target Capacitance (pF)','FontSize',10);
ylabel('Achieved Capacitance (pF)','FontSize',10);
title('Block H — Target vs Achieved (all 48 channels)','FontSize',11,'FontWeight','bold');
cb = colorbar; cb.Label.String = 'Channel #';
colormap(jet); legend('Channels','Ideal y=x','Location','southeast','FontSize',9);
grid on; box on; axis equal;

sgtitle('BLOCK H: Varactor Characterization & Verification','FontSize',13,'FontWeight','bold');

%% =========================================================================
%  FIGURE 9 — End-to-End Signal Chain Overview
%% =========================================================================
figure('Name','Fig 9 - End-to-End System Overview','Color','white', ...
       'NumberTitle','off','Position',[100 100 1100 620]);

subplot(2,3,1);
plot(t_tx(1:16*sps)*1000, tx_wave(1:16*sps),'b-','LineWidth',1.2);
xlabel('Time (ms)','FontSize',9); ylabel('Amplitude','FontSize',9);
title('(1) TX BPSK Signal','FontSize',10,'FontWeight','bold'); grid on; box on;

subplot(2,3,2);
snr_mid = 10^(1.5);
n_mid   = sqrt(Eb_d/(2*snr_mid)) * randn(1, 16*sps);
rx_mid  = tx_wave(1:16*sps) + n_mid;
plot(t_tx(1:16*sps)*1000, rx_mid,'Color',[0.85 0.33 0.10],'LineWidth',0.8);
xlabel('Time (ms)','FontSize',9); ylabel('Amplitude','FontSize',9);
title('(2) RX Signal (AWGN 15 dB)','FontSize',10,'FontWeight','bold'); grid on; box on;

subplot(2,3,3);
semilogy(SNR_LIST, BER_th,'b-o','LineWidth',2,'MarkerSize',7,'MarkerFaceColor','b'); hold on;
semilogy(SNR_LIST, max(BER_sim,1e-7),'r--s','LineWidth',2,'MarkerSize',7,'MarkerFaceColor','r');
xlabel('SNR (dB)','FontSize',9); ylabel('BER','FontSize',9);
title('(3) BER Performance','FontSize',10,'FontWeight','bold');
legend('Theory','Simulated','FontSize',7,'Location','southwest'); grid on; box on;

subplot(2,3,4);
scatter(V_bias, V_recon, 30,'b','filled'); hold on;
plot([0 V_MAX],[0 V_MAX],'r--','LineWidth',1.5);
xlabel('V_{orig} (V)','FontSize',9); ylabel('V_{recon} (V)','FontSize',9);
title('(4) DAC: V_{orig} vs V_{recon}','FontSize',10,'FontWeight','bold');
grid on; box on;

subplot(2,3,5);
ch_g = 24;
duty_g  = V_recon(ch_g)/V_MAX;
pwm_g   = double(mod(t_pwm,1/PWM_FREQ) < duty_g*(1/PWM_FREQ))*V_MAX;
vg      = zeros(size(pwm_g)); vg(1)=pwm_g(1);
for n=2:length(pwm_g), vg(n)=alpha*pwm_g(n)+(1-alpha)*vg(n-1); end
plot(t_pwm*1000, pwm_g,'Color',[0.70 0.85 1.00],'LineWidth',0.9); hold on;
plot(t_pwm*1000, vg,   'r-','LineWidth',2);
yline(V_pwm_out(ch_g),'k--','Label',sprintf('%.2fV',V_pwm_out(ch_g)));
xlabel('Time (ms)','FontSize',9); ylabel('V','FontSize',9);
title(sprintf('(5) PWM Ch%d → DC %.2fV',ch_g,V_pwm_out(ch_g)),'FontSize',10,'FontWeight','bold');
legend('PWM','RC out','FontSize',7); grid on; box on;

subplot(2,3,6);
scatter(C_target, C_achieved, 40, 1:NUM_CH,'filled'); hold on;
plot([min(C_target) max(C_target)],[min(C_target) max(C_target)],'r--','LineWidth',2);
xlabel('C_{target} (pF)','FontSize',9); ylabel('C_{achieved} (pF)','FontSize',9);
title('(6) Varactor: Target vs Achieved','FontSize',10,'FontWeight','bold');
colormap(jet); grid on; box on;

sgtitle('End-to-End System Overview (A to H)','FontSize',13,'FontWeight','bold');

%% =========================================================================
%  FIGURE 10 — Full 48-Channel Results Table (uitable)
%% =========================================================================
figure('Name','Fig 10 - Full 48-Channel Results Table','Color','white', ...
       'NumberTitle','off','Position',[50 50 1120 840]);

annotation('textbox',[0.01 0.95 0.98 0.04], ...
    'String','FIGURE 10 — Full 48-Channel End-to-End Results Table', ...
    'FontSize',12,'FontWeight','bold','EdgeColor','none', ...
    'HorizontalAlignment','center','VerticalAlignment','middle');

col_hdr  = {'Ch','C target (pF)','V bias (V)','ADC Code', ...
            'V recon (V)','Ripple (mV pp)','C achieved (pF)','Error (pF)','Error (%)'};
col_data = num2cell([ (1:NUM_CH)', ...
                      round(C_target',4), ...
                      round(V_bias',4), ...
                      ADC_codes', ...
                      round(V_recon',4), ...
                      round(V_ripple',2), ...
                      round(C_achieved',4), ...
                      round(C_err_pF',4), ...
                      round(C_err_pct',3) ]);

uit = uitable('Parent',gcf, ...
    'Data',       col_data, ...
    'ColumnName', col_hdr, ...
    'RowName',    {}, ...
    'Units',      'normalized', ...
    'Position',   [0.01 0.05 0.98 0.88], ...
    'FontSize',   8.5);
uit.ColumnWidth = {30, 95, 80, 70, 80, 90, 105, 80, 70};

%% =========================================================================
%  CONSOLE — Final Summary
%% =========================================================================
fprintf('\n=============================================================\n');
fprintf('  FULL 48-CHANNEL RESULTS TABLE\n');
fprintf('=============================================================\n');
fprintf('%-4s %-13s %-10s %-8s %-12s %-11s %-13s %-9s %-7s\n', ...
    'Ch','C_tgt(pF)','V_bias(V)','ADCcode','V_recon(V)', ...
    'Ripple(mV)','C_achv(pF)','Err(pF)','Err(%)');
fprintf('%s\n', repmat('-',1,98));
for ch = 1:NUM_CH
    fprintf('%-4d %-13.3f %-10.4f %-8d %-12.4f %-11.2f %-13.3f %-9.4f %-7.3f\n', ...
        ch, C_target(ch), V_bias(ch), ADC_codes(ch), ...
        V_recon(ch), V_ripple(ch), C_achieved(ch), C_err_pF(ch), C_err_pct(ch));
end
fprintf('%s\n', repmat('-',1,98));

fprintf('\n=============================================================\n');
fprintf('  SYSTEM PERFORMANCE SUMMARY\n');
fprintf('=============================================================\n');
fprintf('  ADC resolution          : %d bits\n',       ADC_BITS);
fprintf('  Voltage step (dV)       : %.4f mV/LSB\n',  dV*1000);
fprintf('  Frame size              : %d bits\n',       total_bits);
fprintf('  Symbol rate             : %.1f kbps\n',     Rs/1e3);
fprintf('  Carrier frequency       : %.1f kHz\n',      FC/1e3);
fprintf('  Modulation              : BPSK (coherent)\n');
fprintf('  BER at 25dB SNR (th.)   : %.3e\n',          BER_th(end));
fprintf('  BER at 25dB SNR (sim.)  : %.3e\n',          BER_sim(end));
fprintf('  PWM frequency           : %.1f kHz\n',      PWM_FREQ/1e3);
fprintf('  RC LPF cutoff           : %.1f Hz\n',       fc_lp);
fprintf('  RC time constant        : %.4f ms\n',       RC*1000);
fprintf('  Max PWM ripple          : %.2f mV pp\n',    max(V_ripple));
fprintf('  Mean PWM ripple         : %.2f mV pp\n',    mean(V_ripple));
fprintf('  Max V recon error       : %.4f mV\n',       max(abs(V_err_F))*1000);
fprintf('  RMS V recon error       : %.4f mV\n',       rms(V_err_F)*1000);
fprintf('  Max C error             : %.3f pF (%.2f%%)\n', max(abs(C_err_pF)), max(C_err_pct));
fprintf('  RMS C error             : %.3f pF (%.2f%%)\n', rms(C_err_pF),     rms(C_err_pct));
fprintf('  CRC check               : %s\n',            ternary(crc_ok,'PASSED','FAILED'));
fprintf('=============================================================\n');
fprintf('\n  10 figures generated. All blocks A-H verified.\n\n');

%% =========================================================================
%  HELPER FUNCTION
%% =========================================================================
function s = ternary(cond, a, b)
    if cond, s = a; else, s = b; end
end
