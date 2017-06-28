%%%
% SummerAnalysis.m
% Script used to begin data analysis of summer 2017 experiment
%%%
%% cleaning
clear; close all; clc;
%% load data
data= xlsread('SummerData_Truncated.xlsx','Keith');
%% separate data by latency
j= 0;
k= 0;
for i= 1:length(data)
    if data(i,2) == 0 %low latency
        j= j+1;
        dataLow(j)= data(i,5);
    elseif data(i,2) == 1 %high latency
        k= k+1;
        dataHigh(k)= data(i,5);
    end
end
dataLow= dataLow';
dataHigh= dataHigh';
lowMTD= mean(dataLow);
highMTD= mean(dataHigh);
lowMedian= median(dataLow);
highMedian= median(dataHigh);
%% test normallity
normAll = kstest(data(:,5));
normLow= kstest(dataLow);
normHigh= kstest(dataHigh);
%% plot histograms
figure(1)
histogram(data(:,5),'BinWidth',50);
xlabel('Time to Discovery');
title('Entire Data Set');
figure(2)
subplot(2,1,1);
histogram(dataLow,'BinWidth',50);
title('0.3 second latency');
axis([0 1400 0 8]);
subplot(2,1,2);
histogram(dataHigh,'BinWidth',50);
title('2.6 second latency');
xlabel('Time to Discovery');
axis([0 1400 0 8]);
%% run ANOVA on variance of latency
latency= cell(44,1);
latency(1:22)= {'Low'};
latency(23:44)= {'High'};
varLow= (lowMTD-dataLow).^2;
varHigh= (highMTD-dataHigh).^2;
varOrdered= [varLow;varHigh];
pVar= anova1(varOrdered,latency);
%% run ANOVA on MTD of latency

%lowADM= abs(lowMedian-dataLow);
%highADM= abs(highMedian-dataHigh);
dataOrdered= [dataLow;dataHigh];
pMTD= anova1(dataOrdered,latency);