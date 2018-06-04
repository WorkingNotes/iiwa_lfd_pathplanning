

% get path for tutorial
lfd_path = which('setup_lfd_tutorial');
lfd_path = fileparts(lfd_path);
% remove any auxiliary folder from the search path
restoredefaultpath();
% remove the default user-specific path
userpath('clear');
% add only the lfd path
addpath(genpath(lfd_path));

