function acqResults = acquisition(longSignal, settings)
% Function performs cold start acquisition on the collected "data". It
% searches for COMPASS signals of all satellites, which are listed in field
% "acqSatelliteList" in the settings structure. Function saves code phase
% and frequency of the detected signals in the "acqResults" structure.
% 
% acqResults = acquisition(longSignal, settings)
% 
%    Inputs:
%        longSignal    - 11 ms of raw signal from the front-end 
%        settings      - Receiver settings. Provides information about
%                        sampling and intermediate frequencies and other
%                        parameters including the list of the satellites to
%                        be acquired.
%    Outputs:
%        acqResults    - Function saves code phases and frequencies of the 
%                        detected signals in the "acqResults" structure. The
%                        field "carrFreq" is set to 0 if the signal is not
%                        detected for the given PRN number. 
% 

% Initialization =========================================================

% Find number of samples per spreading code
samplesPerCode = round(settings.samplingFreq / (settings.codeFreqBasis / settings.codeLength));

% Create two "settings.acqCohIntegration" msec vectors of data
% to correlate with:
signal1 = longSignal(1:2*samplesPerCode);

% Find sampling period:
ts = 1 / settings.samplingFreq;

% Find phase points of the local carrier wave:
phasePoints = (0 : (2*samplesPerCode-1)) * 2 * pi * ts;

% Number of the frequency bins for the given acquisition band 
numberOfFrqBins = round(settings.acqSearchBand * 2*1) + 1;

% Generate all ranging codes and sample them according to the sampling freq:
b1CodesTable = makeB1Table(settings);

%--- Initialize arrays to speed up the code -------------------------------
% Search results of all frequency bins and code shifts (for one satellite)
results     = zeros(numberOfFrqBins, 1*samplesPerCode);
% Carrier frequencies of the frequency bins
frqBins     = zeros(1, numberOfFrqBins);
  
%--- Initialize acqResults ------------------------------------------------
% Carrier frequencies of detected signals
acqResults.carrFreq     = zeros(1, 37);
% C/A code phases of detected signals
acqResults.codePhase    = zeros(1, 37);
% Correlation peak ratios of the detected signals
acqResults.peakMetric   = zeros(1, 37);

printf('(');

% Perform search for all listed PRN numbers ...
for PRN = settings.acqSatelliteList      % l'ho modificato a [1:37]

% Correlate signals ======================================================   
  %--- Perform DFT of C/A code ------------------------------------------
  %Multiply B1 code on Neumann-Hoffman code (first 5 ms - otherwise too long wait time)
  %NH = "0, 0, 0, 0, 0, 1, 0, 0, 1, 1, 0, 1, 0, 1,  0,  0,  1,  1,  1,  0";
  b1CodeFreqDom = conj(fft([b1CodesTable(PRN, :) zeros(1, samplesPerCode)]));

  %--- Make the correlation for whole frequency band (for all freq. bins)
  for frqBinIndex = 1:numberOfFrqBins
    %--- Generate carrier wave frequency grid (freqency step depends
    % on "settings.acqCohIntegration") --------------------------------
    frqBins(frqBinIndex) = settings.IF - (settings.acqSearchBand/2) * 1000 + (1000 / (2*1)) * (frqBinIndex - 1);
    %--- Generate local sine and cosine -------------------------------
    sigCarr = exp(i*frqBins(frqBinIndex) * phasePoints);
    
    %--- "Remove carrier" from the signal and Convert the baseband 
    % signal to frequency domain --------------------------------------
    %pause;
    IQfreqDom1 = fft(sigCarr .* signal1);
    
    %--- Multiplication in the frequency domain (correlation in time domain)
    convCodeIQ1 = IQfreqDom1 .* b1CodeFreqDom;

    %--- Perform inverse DFT and store correlation results ------------
    acqRes1 = abs(ifft(convCodeIQ1)) .^ 2;
    
    %--- Check which msec had the greater power and save that, will
    %"blend" 1st and 2nd "settings.acqCohIntegration" msec but will
    % correct data bit issues
    results(frqBinIndex, :) = acqRes1(1:samplesPerCode);
    
  end % frqBinIndex = 1:numberOfFrqBins

% Look for correlation peaks in the results ==============================
  % Find the highest peak and compare it to the second highest peak
  % The second peak is chosen not closer than 1 chip to the highest peak
  
  %--- Find the correlation peak and the carrier frequency --------------
  [peakSize frequencyBinIndex] = max(max(results, 'c'));

  %--- Find code phase of the same correlation peak ---------------------
  [peakSize codePhase] = max(max(results, 'r'));

  %--- Find 1 chip wide CA code phase exclude range around the peak ----
  samplesPerCodeChip   = round(settings.samplingFreq / settings.codeFreqBasis);
  excludeRangeIndex1 = codePhase - samplesPerCodeChip;
  excludeRangeIndex2 = codePhase + samplesPerCodeChip;

  %--- Correct C/A code phase exclude range if the range includes array
  %boundaries
  if excludeRangeIndex1 < 2
      codePhaseRange = excludeRangeIndex2 : (samplesPerCode + excludeRangeIndex1);
  elseif excludeRangeIndex2 > samplesPerCode
      codePhaseRange = (excludeRangeIndex2 - samplesPerCode) : excludeRangeIndex1;
  else
      codePhaseRange = [1:excludeRangeIndex1, excludeRangeIndex2 : samplesPerCode];
  end
  
  %--- Find the second highest correlation peak in the same freq. bin ---
  secondPeakSize = max(results(frequencyBinIndex, codePhaseRange));

  %--- Store result -----------------------------------------------------
  acqResults.peakMetric(PRN) = peakSize/secondPeakSize;
  
  % If the result is above threshold, then there is a signal ...
  if (peakSize/secondPeakSize) > settings.acqThreshold
    %--- Indicate PRN number of the detected signal -------------------
    printf('%02d ', PRN);
    acqResults.codePhase(PRN) = codePhase;
    acqResults.carrFreq(PRN)  = settings.IF - (settings.acqSearchBand/2) * 1000 + (1000 / (2*1)) * (frequencyBinIndex - 1);
      
  else
    %--- No signal with this PRN --------------------------------------
    printf('. ');
  end   % if (peakSize/secondPeakSize) > settings.acqThreshold
    
end    % for PRN = satelliteList

%=== Acquisition is over ==================================================
printf(')\n');

