function setup_course(section)
%SETUP_COURSE Select hw1, hw2 or project and configure MATLAB search paths.
% Example: setup_course('hw1'); hw1_part1;
    arguments
        section (1,:) char {mustBeMember(section, {'hw1', 'hw2', 'project'})}
    end

    repoRoot = fileparts(mfilename('fullpath'));
    addpath(repoRoot);
    switch section
        case 'hw1'
            sectionRoot = fullfile(repoRoot, 'assignments', 'assignment-01');
        case 'hw2'
            sectionRoot = fullfile(repoRoot, 'assignments', 'assignment-02');
        case 'project'
            sectionRoot = fullfile(repoRoot, 'project', 'double-tank');
    end

    folders = {'src', 'data', 'controllers', 'dependencies'};
    for i = 1:numel(folders)
        folder = fullfile(sectionRoot, folders{i});
        if isfolder(folder)
            addpath(folder);
        end
    end
    cd(sectionRoot);
    fprintf('Working folder: %s\nSee README.md for experiment order.\n', sectionRoot);
end
