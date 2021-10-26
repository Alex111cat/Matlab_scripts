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


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%% Receiver code %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
phyMode = 'LE2M';
bleParam = helperBLEReceiverConfig(phyMode);

signalSource = 'ADALM-PLUTO'; % The default signal source is 'File'

if strcmp(signalSource,'File')
    switch bleParam.Mode
        case 'LE1M'
            bbFileName = 'bleCapturesLE1M.bb';
        case 'LE2M'
            bbFileName = 'bleCapturesLE2M.bb';
        case 'LE500K'
            bbFileName = 'bleCapturesLE500K.bb';
        case 'LE125K'
            bbFileName = 'bleCapturesLE125K.bb';
        otherwise
            error('Invalid PHY transmission mode. Valid entries are LE1M, LE2M, LE500K and LE125K.');
    end
    sigSrc = comm.BasebandFileReader(bbFileName);
    sigSrcInfo = info(sigSrc);
    sigSrc.SamplesPerFrame = sigSrcInfo.NumSamplesInData;
    bbSampleRate = sigSrc.SampleRate;
    bleParam.SamplesPerSymbol = bbSampleRate/bleParam.SymbolRate;

elseif strcmp(signalSource,'ADALM-PLUTO')

    % First check if the HSP exists
    if isempty(which('plutoradio.internal.getRootDir'))
        error(message('comm_demos:common:NoSupportPackage', ...
            'Communications Toolbox Support Package for ADALM-PLUTO Radio',...
            ['<a href="https://www.mathworks.com/hardware-support/' ...
            'adalm-pluto-radio.html">ADALM-PLUTO Radio Support From Communications Toolbox</a>']));
    end

    bbSampleRate = bleParam.SymbolRate * bleParam.SamplesPerSymbol;
    sigSrc = sdrrx('Pluto',...
        'RadioID',             'usb:1',...
        'CenterFrequency',     2.402e9,...
        'BasebandSampleRate',  bbSampleRate,...
        'SamplesPerFrame',     1e7,...
        'GainSource',         'Manual',...
        'Gain',                25,...
        'OutputDataType',     'double');
else
    error('Invalid signal source. Valid entries are File and ADALM-PLUTO.');
end

% Setup spectrum viewer
spectrumScope = dsp.SpectrumAnalyzer( ...
    'SampleRate',       bbSampleRate,...
    'SpectrumType',     'Power density', ...
    'SpectralAverages', 10, ...
    'YLimits',          [-130 -30], ...
    'Title',            'Received Baseband BLE Signal Spectrum', ...
    'YLabel',           'Power spectral density');

% The transmitted waveform is captured as a burst
dataCaptures = sigSrc();

% Show power spectral density of the received waveform
spectrumScope(dataCaptures);

% Обработка приемника
% Initialize System objects for receiver processing
agc = comm.AGC('MaxPowerGain',20,'DesiredOutputPower',2);

freqCompensator = comm.CoarseFrequencyCompensator('Modulation','OQPSK', ...
    'SampleRate',bbSampleRate,...
    'SamplesPerSymbol',2*bleParam.SamplesPerSymbol,...
    'FrequencyResolution',100);

prbDet = comm.PreambleDetector(bleParam.RefSeq,'Detections','First');

% Initialize counter variables
pktCnt = 0;
crcCnt = 0;
displayFlag = false; % true if the received data is to be printed

% Loop to decode the captured BLE samples
while length(dataCaptures) > bleParam.MinimumPacketLen

    % Consider two frames from the captured signal for each iteration
    startIndex = 1;
    endIndex = min(length(dataCaptures),2*bleParam.FrameLength);
    rcvSig = dataCaptures(startIndex:endIndex);

    rcvAGC = agc(rcvSig); % Perform AGC
    rcvDCFree = rcvAGC - mean(rcvAGC); % Remove the DC offset
    rcvFreqComp = freqCompensator(rcvDCFree); % Estimate and compensate for the carrier frequency offset
    rcvFilt = conv(rcvFreqComp,bleParam.h,'same'); % Perform gaussian matched filtering

    % Perform frame timing synchronization
    [~, dtMt] = prbDet(rcvFilt);
    release(prbDet)
    prbDet.Threshold = max(dtMt);
    prbIdx = prbDet(rcvFilt);

    % Extract message information
    [cfgLLAdv,pktCnt,crcCnt,remStartIdx] = helperBLEPhyBitRecover(rcvFilt,...
        prbIdx,pktCnt,crcCnt,bleParam);

    % Remaining signal in the burst captures
    dataCaptures = dataCaptures(1+remStartIdx:end);

    % Display the decoded information
    if displayFlag && ~isempty(cfgLLAdv)
        fprintf('Advertising PDU Type: %s\n',cfgLLAdv.PDUType);
        fprintf('Advertising Address: %s\n',cfgLLAdv.AdvertiserAddress);
    end

    % Release System objects
    release(freqCompensator)
    release(prbDet)
end

% Release the signal source
release(sigSrc)

% Determine the PER
if pktCnt
    per = 1-(crcCnt/pktCnt);
    fprintf('Packet error rate for %s mode is %f.\n',bleParam.Mode,per);
else
    fprintf('\n No BLE packets were detected.\n')
end
