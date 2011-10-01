function spectrum_analysis(x,fs,duration)

figure(1);

subplot(2,1,1);
t=(0:1:length(x)-1)/fs;
plot(t,x);
axis([0, duration, -1, 1]);
xlabel('Time (s)');
ylabel('Amplitude');
axis([0,duration,min(x)*1.1,max(x)*1.1]);

subplot(2,1,2);
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

figure(2);

input("Press any key...");