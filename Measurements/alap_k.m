function alap_k(X1, YMatrix1)
%CREATEFIGURE(X1, YMATRIX1)
%  X1:  vector of x data
%  YMATRIX1:  matrix of y data

%  Auto-generated by MATLAB on 13-Dec-2016 19:30:57

% Create figure
figure1 = figure('PaperSize',[15 15]);

% Create axes
axes1 = axes('Parent',figure1);
hold(axes1,'on');

% Create multiple lines using matrix input to loglog
loglog1 = loglog(X1,YMatrix1,'MarkerSize',4,'LineWidth',1.2,'Parent',axes1,...
    'LineStyle','none');
set(loglog1(1),'DisplayName','$M\acute{e}r\acute{e}si\;eredm\acute{e}nyek$','Marker','diamond',...
    'Color',[0 0 0],...
    'LineStyle','-');

% Create xlabel
xlabel('$z/M\;[-]$','FontSize',16,'FontName','Palatino Linotype',...
    'Interpreter','latex');

% Create ylabel
ylabel('$k\;[\frac{m^2}{s^2}]$','FontSize',16,...
    'FontName','Palatino Linotype',...
    'Interpreter','latex');

box(axes1,'on');
grid(axes1,'on');
axis(axes1,'square');
% Set the remaining axes properties
set(axes1,'GridAlpha',1,'MinorGridAlpha',1,'XMinorTick','on','XScale','log',...
    'XTickLabel',{'0.1','1','10','100'},'YMinorTick','on','YScale','log',...
    'YTickLabel',{'0.01','0.1','1','10','100'});
% Create legend
legend1 = legend(axes1,'show');
set(legend1,'Location','southwest','Interpreter','latex');

