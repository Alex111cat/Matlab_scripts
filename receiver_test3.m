clc;
clear all;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%% Transmitter Code %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Настройка рекламного канала PDU
cfgLLAdv = bleLLAdvertisingChannelPDUConfig;
cfgLLAdv.PDUType = 'Advertising indication';
cfgLLAdv.AdvertisingData = '0123456789ABCDEF';
cfgLLAdv.AdvertiserAddress = '1234567890AB';

% Создание рекламного канала PDU
messageBits = bleLLAdvertisingChannelPDU(cfgLLAdv);

phyMode = 'LE2M'; % Select one mode from the set {'LE1M','LE2M','LE500K','LE125K'}
sps = 8; % Samples per symbol
channelIdx = 37; % Channel index value in the range [0,39]
accessAddLen = 32;% Length of access address
accessAddHex = '8E89BED6'; % Access address value in hexadecimal
accessAddBin = de2bi(hex2dec(accessAddHex),accessAddLen)'; % Access address in binary

symbolRate = 2e6;

% Generate BLE waveform
txWaveform = bleWaveformGenerator(messageBits,...
    'Mode', phyMode,...
    'SamplesPerSymbol',sps,...
    'ChannelIndex', channelIdx,...
    'AccessAddress', accessAddBin);

% Setup spectrum viewer
spectrumScope = dsp.SpectrumAnalyzer( ...
    'SampleRate', symbolRate*sps,...
    'SpectrumType', 'Power density', ...
    'SpectralAverages', 10, ...
    'YLimits', [-130 0], ...
    'Title', 'Baseband BLE Signal Spectrum', ...
    'YLabel', 'Power spectral density');

% Show power spectral density of the BLE signal
spectrumScope(txWaveform);


% Cоздание объекта настройки для угловой оценки BLE
cfg = bleAngleEstimateConfig('ArraySize',4); 
% IQsamples = [0.8507+0.5257i;-0.5257 + 0.8507i;-0.8507 - 0.5257i;...
%     0.5257 - 0.8507i;0.8507+0.5257i;-0.5257 + 0.8507i;...
%     -0.8507 - 0.5257i;0.5257 - 0.8507i;-0.3561 + 0.9345i;-0.3561 + 0.9345i];

%Выборки IQ в виде вектор-столбца с комплексным знаком.
%Этот аргумент соответствует 8 μs значениям длительности паза и отчетного периода
IQsamples = txWaveform;

IQsamples2 = IQsamples * 0;
k = 1;
for i = 1:1536
    if mod(i, 15) == 0
      k = k + 1;  
      IQsamples2(k, 1) = IQsamples(i); 
    end
end

IQsamples2(1:k)
% spectrumScope(IQsamples2(1:1536));
%Оценка угола прибытия (AoA) или угола отбытия (AoD)
angle = bleAngleEstimate(IQsamples2(1:k),cfg)

% Обработка передатчика
% Initialize the parameters required for signal source
txCenterFrequency       = 2.402e9;  % Varies based on channel index value
txFrameLength           = length(txWaveform);
txNumberOfFrames        = 1e4;
txFrontEndSampleRate    = symbolRate*sps;

% The default signal source is 'File'
signalSink = 'ADALM-PLUTO';

if strcmp(signalSink,'File')

    sigSink = comm.BasebandFileWriter('CenterFrequency',txCenterFrequency,...
        'Filename','bleCaptures.bb',...
        'SampleRate',txFrontEndSampleRate);
    sigSink(txWaveform); % Writing to a baseband file 'bleCaptures.bb'

elseif strcmp(signalSink,'ADALM-PLUTO')

    % First check if the HSP exists
    if isempty(which('plutoradio.internal.getRootDir'))
        error(message('comm_demos:common:NoSupportPackage', ...
                      'Communications Toolbox Support Package for ADALM-PLUTO Radio',...
                      ['<a href="https://www.mathworks.com/hardware-support/' ...
                      'adalm-pluto-radio.html">ADALM-PLUTO Radio Support From Communications Toolbox</a>']));
    end
    connectedRadios = findPlutoRadio; % Discover ADALM-PLUTO radio(s) connected to your computer
    radioID = connectedRadios(1).RadioID;
    sigSink = sdrtx( 'Pluto',...
        'RadioID',           'usb:0',...
        'CenterFrequency',   txCenterFrequency,...
        'Gain',              0,...
        'SamplesPerFrame',   txFrameLength,...
        'BasebandSampleRate',txFrontEndSampleRate);
    % The transfer of baseband data to the SDR hardware is enclosed in a
    % try/catch block. This means that if an error occurs during the
    % transmission, the hardware resources used by the SDR System
    % object(TM) are released.
    currentFrame = 1;
    try
        while currentFrame <= txNumberOfFrames
            % Data transmission
            sigSink(txWaveform);
            % Update the counter
            currentFrame = currentFrame + 1;
        end
    catch ME
        release(sigSink);
        rethrow(ME)
    end
else
    error('Invalid signal sink. Valid entries are File and ADALM-PLUTO.');
end

% Release the signal sink
release(sigSink)
