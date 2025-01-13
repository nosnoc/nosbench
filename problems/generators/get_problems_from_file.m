function problems = get_problems_from_file(problem_list_file)
    problems = [];
    problem_paths = [];
    problem_lines = readlines(problem_list_file);
    problems_path = getenv('NOSBENCH_PROBLEM_DIR');
    for problem_line=problem_lines'
        problem_path = fullfile(problems_path, problem_line);
        problem_paths = [problem_paths, problem_path];
    end

    problem_files = [];
    for problem_path=problem_paths
        problem_file_list = dir(problem_path);
        problem_file_list = problem_file_list(~ismember({problem_file_list.name},{'.','..'}));
        for problem_file=problem_file_list'
            problem_files = [problem_files, string(fullfile(problem_file.folder,problem_file.name))];
        end
    end
    size(problem_files)

    for problem_file=problem_files
        disp(problem_file)
        problem = load(problem_file);
        [~,name,ext] = fileparts(problem_file);
        problem.name = char(name);
        problems = [problems, problem];
    end
end
