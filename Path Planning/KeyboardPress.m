function [] = move_fig()
% move figure with arrow keys.
S.fh = figure('units','pixels',...
              'position',[500 500 200 260],...
              'menubar','none',...
              'name','move_fig',...
              'numbertitle','off',...
              'resize','off',...
              'keypressfcn',@fh_kpfcn);
            
function [] = fh_kpfcn(H,E)          
% Figure keypressfcn
switch E.Key
    case 'rightarrow'
        disp('right')
    case 'leftarrow'
        disp('left')
    case 'uparrow'
        disp('up')
    case 'downarrow'
        disp('down')
    otherwise  
end