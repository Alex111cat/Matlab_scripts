clc;
clear all;

% Передача данных с использованием объекта радиосистемы ADALM-PLUTO
tx = sdrtx('Pluto', ...
            'RadioID','usb:0', ...
            'CenterFrequency',2.4e9, ...
            'BasebandSampleRate',1e6);
       
dpskMod = comm.DPSKModulator('BitInput',true);
% Передача данных
for counter = 1:20
   data = randi([0 1],30,1);
   modSignal=dpskMod(data);
   tx(modSignal);
end
data
writematrix(data,'Data_TX.txt','Delimiter',';')  
release(tx);

% Получение данных с помощью объекта радиосистемы ADALM-PLUTO
rx = sdrrx('Pluto','RadioID','ip:192.168.2.1');
data1 = rx();
data1
%capture(rx,5,'Seconds','Filename','RXRecording.bb');
writematrix(data1,'Data_RX.txt','Delimiter',';')  
release(rx);