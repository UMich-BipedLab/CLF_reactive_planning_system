clc, clear
fig = createFigureOptions(2, 1, "rand_nums", 1, 1, 12);
[axes_handles, fig_handles] = createFigHandleWithOptions(fig);

assign_path = 3;
assign_file_name = 0;
file_name = "2021-20-06-17-44-07_random_numbers.csv";

if assign_path == 1
    path = "/home/brucebot/workspace/catkin/bagfiles/autonomy2021/June04/";
elseif assign_path == 2
    path = "/home/brucebot/.ros/";
elseif assign_path == 3
    path = "/home/brucebot/workspace/catkin/";
else
    path = "./";
end



% cd(path)
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
% cur_fig = 1;          
% popCurrentFigure(fig_handles(cur_fig))
% hold off
% x = 1 : size(T, 1);
% x = x ./ (300);
% h = stackedplot(x,table2array(T(:,1:2)), ...
%     'Title', strrep(file_name, '_', ' '));
% h.DisplayLabels = T.Properties.VariableNames(1:2)';

[axes_h, fig_h] = getCurrentFigure(1, axes_handles, fig_handles);      
h = histogram(axes_h, table2array(T(:,2)));
h.BinWidth = 0.2;