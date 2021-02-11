function [ fig, ax ] = plotPublication(pattern,xValue,yValue, xLabel, yLabel, figName, randomizeSeed, saveFig, fileName)
% PLOTPUBLICATION to plot comparison figure for publication 
%   Arguments:
%       pattern: a cell array of strings for the legend.
%               Also used for computing the number of plots in the figure
%       xValue, yValue: a cell array of xValues, yValues. Cell index corresponds to the pattern
%       saveFig: a boolean value to save figure or not
%       fileName: file name of the figure to save
%       xLabel, yLabel, figName: string values for the corresponding in the fig
%       randomizeSeed: the seed to randomize the colors and markers between comparison figures. 
%                       Entering a seed means we try to keep the colors and markers same for the same pattern sequence. Ideal for different plots
%   Returns:
%       fig: figure handle
%       ax: axis handle
%   Usage:
%       plotPublication(pattern,xValue,yValue) --> 3 arguments (MINIMUM)
%       plotPublication(pattern,xValue,yValue,yLabel) --> 4 arguments
%       plotPublication(pattern,xValue,yValue,xLabel,yLabel,figName,saveFig,fileName)
% 
%   Author: Sajid Mohamed (s.mohamed@tue.nl)

%   Organization: Eindhoven University of Technology
%   Date: January 14, 2020
%% Default argument values
if nargin < 3
    error('Not enough input arguments for plotting the comparison. For details - type >>help plotPublication');
end
if nargin < 4
    yLabel='yL (m)';
end
if nargin < 5
    xLabel='time (s)';
end
if nargin < 6
    figName='yL';
end
if nargin < 7
    randomizePlots=0;
else
    randomizePlots=1;
end
if nargin < 8
    saveFig=0;
end
if nargin < 9
    fileName=sprintf('res_%s',figName);
end
%% initialisation of parameters
width = 10;     % Width in inches
height = 6;    % Height in inches
alw = 0.75;    % AxesLineWidth
fsz = 20;      % Fontsize
lw = 1;      % LineWidth
msz = 4;       % MarkerSize
plot_colours=['g' 'b' 'k'];% 'r' 'm' 'c' 'y' 'g' 'b' 'k' 'r' 'm' 'c']; 
plot_markers=['o' 'x' '+'];% '*' '.' 's' 'd' 'p' 'h' 'v' '^' '<' '>'];
if size(plot_colours)~=size(plot_markers)
    error('Size of plot_colours and plot_markers should be same');
end
plot_markerTypes={'-', '--', ':', '-.'};%solid (default), dashed, dotted,dash-dot
%BEGIN: when pattern size more than colours. Hack!
markerType={};
for i=1:length(plot_markers)    
    markerType{end+1}=strcat(plot_markerTypes{1},plot_markers(i));%sprintf('%s%c',plot_markerTypes(1),plot_markers(i))];
end
checkSize=ceil(length(pattern)/length(plot_colours));
i=1;plot_colours1=plot_colours;
while i<checkSize
    plot_colours1=[plot_colours1 plot_colours];
    index=length(plot_markers);
    for j=1:index %Enforcing a moving window so that markers and colours combination are mostly not identical during repetition
        if (i+j)<=index
            markerType{end+1}=strcat(plot_markerTypes{mod(i,length(plot_markerTypes))+1},plot_markers(i+j));
        else
            markerType{end+1}=strcat(plot_markerTypes{mod(i,length(plot_markerTypes))+1},plot_markers(mod(i-1+j,index)+1));
        end
    end
    i=i+1;
end
plot_colours=plot_colours1;
%END: when pattern size more than colours. Hack!
%% Randomize markers and colours keeping the seed constant
if randomizePlots    
    rng(randomizeSeed);
    plot_colours = plot_colours(randperm(length(plot_colours)));
    rng(randomizeSeed);
    markerType = markerType(randperm(length(markerType)));
end
%% plot figure
figure('name',figName)
pos = get(gcf, 'Position');
%set(gcf, 'Position', [pos(1) pos(2) width*100, height*100]); %<- Set size
set(gca, 'FontSize', fsz, 'LineWidth', alw); %<- Set properties
for loop=1:length(pattern)
    %marker_type=strcat('-',plot_markers(loop));
    plot(xValue{loop}, yValue{loop},markerType{loop},'LineWidth',lw,'MarkerSize',msz,...
        'MarkerEdgeColor',plot_colours(loop),...
        'Color', plot_colours(loop))
    hold on
end   
ylabel(yLabel,'FontSize',20) % x-axis label
xlabel(xLabel,'FontSize',20) % y-axis label
legend(pattern,'Location','southeast','FontSize',20); 
set(gcf,'InvertHardcopy','on');
set(gcf,'PaperUnits', 'inches');
papersize = get(gcf, 'PaperSize');
left = (papersize(1)- width)/2;
bottom = (papersize(2)- height)/2;
myfiguresize = [left, bottom, width, height];
set(gcf,'PaperPosition', myfiguresize);
hold off
%% Get return values (figure and axis handles)
fig = gcf;
ax = gca;
%% Save the file as PNG
if saveFig
    print(fileName,'-dpng','-r300');
end