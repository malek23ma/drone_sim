function whiten(fig)
%WHITEN  Force a figure (and all its axes) to a clean white/light look,
%        overriding MATLAB's dark theme. Call after plotting: whiten(gcf).
    if nargin < 1 || isempty(fig), fig = gcf; end
    try, theme(fig,'light'); catch, end          % R2025a+ themed figures
    set(fig,'Color','w');
    axs = findall(fig,'Type','axes');
    for k = 1:numel(axs)
        a = axs(k);
        set(a,'Color','w','XColor','k','YColor','k','ZColor','k', ...
              'GridColor',[0.15 0.15 0.15],'MinorGridColor',[0.3 0.3 0.3]);
        if ~isempty(a.Title),  set(a.Title,'Color','k');  end
        lg = get(a,'Legend');
        if ~isempty(lg), set(lg,'Color','w','TextColor','k'); end
    end
end
