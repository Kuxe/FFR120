clc
clf
close all
clear all

A = load('text.txt');

mus = unique(A(:,1));
vars = unique(A(:,2));

diss = (A(:,4) - mean(A(:,4)))/std(A(:,4));
diss = reshape(diss,[length(mus) length(vars)]);
eff = (A(:,3) - mean(A(:,3)))/std(A(:,3));
eff = reshape(eff,[length(mus) length(vars)]);
[X,Y]= meshgrid(mus,vars);

subplot(1,2,1);
surf(X,Y,eff);
title('Efficiency','interpreter','latex');
xlabel('Mean','interpreter','latex');
ylabel('Variance','interpreter','latex');
subplot(1,2,2);
surf(X,Y,diss);
title('Discomfort','interpreter','latex');
xlabel('Mean','interpreter','latex');
ylabel('Variance','interpreter','latex');