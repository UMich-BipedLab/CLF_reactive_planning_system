clc, clear
fig = createFigureOptions(2, 1, "communication", 1, 1, 12);
[axes_handles, fig_handles] = createFigHandleWithOptions(fig);

assign_path = 1;
assign_file_name = 0;
file_name = "04-06-2021-00-57-04_command_history.csv";

if assign_path
    path = "/home/brucebot/workspace/catkin/bagfiles/autonomy2021/June04/";
else
    path = "/home/brucebot/.ros/";
end



cd(path)
files = "*.csv";
listing = dir(path + files);
num_files = length(listing);

if assign_file_name
    k = find({listing(:).name} == file_name);
    assert(~isempty(k), "No such file: %s", file_name)
else
    k = num_files;
%     k = 3;
end



file_name = listing(k).name;
file_folder = listing(k).folder;
% name = '20-05-2021-19-16-27_command_history.csv';
fprintf("Using file %i/%i\n", k, num_files);
fprintf("Visulizing %s\n", file_name);
T = readtable(convertCharsToStrings(file_folder) + "/" + ...
              convertCharsToStrings(file_name)); 
if isempty(T)
    fprintf("%s is empty\n", file_name)
    return
end
          %%
cur_fig = 1;          
popCurrentFigure(fig_handles(cur_fig))
hold off
x = 1 : size(T, 1);
x = x ./ (300);
h = stackedplot(x,table2array(T(:,1:9)), ...
    'Title', strrep(file_name, '_', ' '));
h.DisplayLabels = T.Properties.VariableNames(1:9)';

%%
clc
line_width = 2;
clf(fig_handles(2))
popCurrentFigure(fig_handles(2))
ax1 = subplot(6,1,1);
plot(ax1, x, table2array(T(:,2)), 'LineWidth', line_width)
xticks(ax1, [])
% 'baseline' | 'top' | 'cap' | 'middle' | 'bottom'.
ylabel(ax1, "$v_x$ [m/s]", 'rotation',90, ...
    'VerticalAlignment','bottom', 'HorizontalAlignment','center')
reloadCurrentPlot(ax1, 0)

ax2 = subplot(6,1,2);
plot(ax2, x, table2array(T(:,3)), 'LineWidth', line_width)
xticks(ax2, [])
ylabel(ax2, "$v_y$ [m/s]", 'rotation',90, ...
    'VerticalAlignment','bottom', 'HorizontalAlignment','center')
reloadCurrentPlot(ax2, 0)

ax3 = subplot(6,1,3);
plot(ax3, x, rad2deg(table2array(T(:,4))), 'LineWidth', line_width)
xticks(ax3, [])
ylabel(ax3, "$\omega$ [deg/s]", 'rotation',90,...
    'VerticalAlignment','bottom', 'HorizontalAlignment','center')
reloadCurrentPlot(ax3, 0)

ax4 = subplot(6,1,4);
plot(ax4, x, table2array(T(:,5)), 'LineWidth', line_width)
xticks(ax4, [])
ylabel(ax4, "$v_r$ [m/s]", 'rotation',90,...
    'VerticalAlignment','bottom', 'HorizontalAlignment','center')
reloadCurrentPlot(ax4, 0)

ax5 = subplot(6,1,5);
plot(ax5, x, rad2deg(table2array(T(:,6))), 'LineWidth', line_width)
xticks(ax5, [])
ylabel(ax5, "$v_\delta$ [deg/s]", 'rotation',90,...
    'VerticalAlignment','bottom', 'HorizontalAlignment','center')
reloadCurrentPlot(ax5, 0)

ax6 = subplot(6,1,6);
plot(ax6, x, rad2deg(table2array(T(:,9))), 'LineWidth', line_width)
reloadCurrentPlot(ax6, 0)
ylabel(ax6, "$\theta$ [deg]", 'rotation',90,...
    'VerticalAlignment','bottom', 'HorizontalAlignment','center')
xlabel(ax6, "time [s]")


linkaxes([ax1,ax2,ax3, ax4, ax5, ax6],'x')


% saveCurrentPlot(fig_handles(2), ...
%     "/home/brucebot/workspace/motion_planning/new_CLF_for_planning/ExpControlCommands", 'png')

% stackedplot(x,table2array(T(:,1:7)), ...
%     {T.Properties.VariableNames(1:7)})
% stackedplot(T(:,1:7), x)