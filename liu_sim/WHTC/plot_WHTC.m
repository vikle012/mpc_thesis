% NAME: plot_WHTC
%
% PURPOSE: Plotting the speed profile and torque profile of the WHTC
%
% OTHER FILES REQUIRED:
%   .m files:
%       none
%
%   .mat files:
%       WHTC_data
%
% SUBFUNCTIONS:
%   none
%
% Author: Unknown
% May 2018; Last revision: 23-May-2018

%------------- BEGIN CODE --------------

load WHTC_data.mat

figure(1)
sp(1)=subplot(211);
pl(1)=plot(time, speed,'-x','ZDataSource','time');
grid
xlabel('Time [s]')
ylabel('Speed [%]')
sp(2)=subplot(212);
pl(2)=plot(time,torque,'-x','ZDataSource','time');
grid
xlabel('Time [s]')
ylabel('Torque [%]')
linkaxes(sp,'x')


fprintf('Percent of time under 25%% torque %3.1f\n',sum(torque<25)/length(torque)*100);
fprintf('Number of times when the torque rise through 25%% is %3.0f times\n',sum(diff(torque<25)==1));

x=25;
a=find(diff(torque<x)==1); 
b=find(diff(torque<x)==-1);
zz=b(1:end)-[0 ;a(1:end-1)];


figure(2)
sp(1)=subplot(221);
pl(3)=plot(speed, torque,'x','ZDataSource','time');
grid
xlabel('Speed [%]')
ylabel('Torque [%]')

sp(2)=subplot(223);
[Y,X]=hist(speed,20);
bar(X,Y/max(Y)*100)
[Y,X]=hist(speed,length(speed));
line(X,cumsum(Y)/sum(Y)*100,'Color','r','LineWidth',4)
grid
xlabel('Speed [%]')

sp(2)=subplot(222);
[Y,X]=hist(torque,21);
barh(X,Y/max(Y)*100)
[Y,X]=hist(torque,length(torque));
line(cumsum(Y)/sum(Y)*100,X,'Color','r','LineWidth',4)
grid
ylabel('Torque [%]')

subplot(224);
bar(sort(zz,'descend'))
mean(zz)
grid
xlim([0 length(zz)])

%------------- END OF CODE --------------
