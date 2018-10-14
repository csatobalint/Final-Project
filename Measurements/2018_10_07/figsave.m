function []=figsave(fig,name,varargin)
% DOCS String

	switch nargin
		case 2 
			height=10; width=10;
		case 4
			width = varargin{1};
			height = varargin{2};
		otherwise
			error=('Nem megfelelõ számú paraméter')	
	end

set(fig, 'PaperUnits', 'Centimeter')
set(fig,'PaperSize',[width,height])
set(fig, 'PaperPosition', [0 0 width height])
%print(fig, '-dpng', name,'-r600')
print(fig, '-dpdf', name)
end