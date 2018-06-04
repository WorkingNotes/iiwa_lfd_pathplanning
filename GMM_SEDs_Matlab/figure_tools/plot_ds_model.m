function h = plot_ds_model(fig, ds, target, varargin)

quality='low';

if ~isempty(varargin)
	quality=varargin{1};
end

if strcmpi(quality,'high')
    nx=600;
    ny=600;
elseif strcmpi(quality,'medium')
    nx=400;
    ny=400;
else
    nx=200;
    ny=200;
end

axlim = [get(gca,'Xlim'), get(gca, 'Ylim')];
ax_x=linspace(axlim(1),axlim(2),nx); %computing the mesh points along each axis
ax_y=linspace(axlim(3),axlim(4),ny); %computing the mesh points along each axis
[x_tmp, y_tmp]=meshgrid(ax_x,ax_y); %meshing the input domain
x=[x_tmp(:), y_tmp(:)]';
xd = feval(ds, x-repmat(target,1,size(x,2)));
h = streamslice(x_tmp,y_tmp,reshape(xd(1,:),ny,nx),reshape(xd(2,:),ny,nx),1,'method','cubic');

end