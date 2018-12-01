function epsilonCompare(X1, YMatrix1, X2, YMatrix2)
%CREATEFIGURE1(X1, YMatrix1, X2, YMatrix2)
%  X1:  vector of x data
%  YMATRIX1:  matrix of y data
%  X2:  vector of x data
%  YMATRIX2:  matrix of y data

%  Auto-generated by MATLAB on 24-Nov-2018 13:47:01

% Create figure
figure1 = figure('PaperSize',[15 15]);

% Create axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');

% Create multiple lines using matrix input to plot
plot1 = plot(X1,YMatrix1);
set(plot1(1),'DisplayName','Numeric Solution','LineWidth',4,...
    'Color',[1 0.400000005960464 0.400000005960464]);
set(plot1(2),'DisplayName','Analyitic Solution','LineWidth',2,...
    'Color',[1 0.843137264251709 0]);

% Create multiple lines using matrix input to plot
plot2 = plot(X2,YMatrix2,'LineWidth',2,'LineStyle',':','Parent',axes1);
set(plot2(1),'DisplayName','Measurement','MarkerSize',3,...
    'Color',[0.313725501298904 0.313725501298904 0.313725501298904]);
set(plot2(2),'DisplayName','Measurement (MA4)',...
    'Color',[0 0.498039215803146 0]);
set(plot2(3),'DisplayName','Measurement (MA10)',...
    'Color',[0 0.447058826684952 0.74117648601532]);

% Create ylabel
ylabel('$\varepsilon~[rad/s^2]$','Interpreter','latex');

% Create xlabel
xlabel('Time [s]','Interpreter','latex');

% Uncomment the following line to preserve the X-limits of the axes
 xlim(axes1,[0 1]);
% Uncomment the following line to preserve the Y-limits of the axes
 ylim(axes1,[0 165]);
box(axes1,'on');
grid(axes1,'on');
axis(axes1,'square');
% Set the remaining axes properties
set(axes1,'FontSize',16,'YTick',[0 25 50 75 100 125 150]);
% Create legend
legend(axes1,'show');
