function spectrum_analysis_complex(x,fs,duration)

figure(1);
X=fft(x);
N=length(x);
ws=2*pi/N;
wnorm=-pi:ws:pi;
wnorm=wnorm(1:length(x));
w=(wnorm*fs)/(2*pi);
fft_shift_abs=abs(fftshift(X));
plot(w,fft_shift_abs);
xlabel('Frequency (Hz)');
ylabel('Amplitude');
axis([w(1),w(length(w)),min(fft_shift_abs)*1.1,max(fft_shift_abs)*1.1]);

input("Press any key...");
