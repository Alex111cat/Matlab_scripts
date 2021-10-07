clc;
clear all;

% настройка прошивки радио ADALM-PLUTO
configurePlutoRadio('AD9364');

% Создание системного объекта приемника SDR с указанными свойствами.
% Указанная центральная частота соответствует местной FM-станции.
deviceName = 'Pluto';
samplerate = 44e3;
fmStationFrequency = 88.7e6;
rx = sdrrx(deviceName,'BasebandSampleRate',samplerate, ...
    'CenterFrequency',fmStationFrequency,'OutputDataType','double');

% Вызов функции capture с указанием объекта-получателя, продолжительности захвата и имени файла.
% После захвата FM-сигнала происходит разблокировка объекта-приемника с помощью функции release.
capture(rx,5,'Seconds','Filename','FMRecording.bb');
release(rx);

% Демодуляция записи FM
% Создание системного объекта для чтения захваченного сигнала и извлечения кадров данных из файла
% Берется 4400 выборок за кадр при чтении сохраненного сигнала
bbr = comm.BasebandFileReader('FMRecording.bb');
bbr.SamplesPerFrame = 4400;

% Демодуляция и воспроизведение каждого кадра данных FM
fmbDemod = comm.FMBroadcastDemodulator('AudioSampleRate',48e3, ...
    'SampleRate',bbr.Metadata.BasebandSampleRate,'PlaySound',true);
while ~isDone(bbr)
    fmbDemod(bbr());
end