function results = load_results(results_dir)
    result_paths = fullfile(results_dir,{dir(fullfile(results_dir, '*.mat')).name});

    n = length(result_paths);
    i = 1;
    f = waitbar(0, "starting");
    results = {};
    for path=result_paths
        waitbar(i/n, f, sprintf('Progress: %d \\%%', floor(i/n*100)));
        i = i+1;
        if(strcmp(fullfile(results_dir, 'data_table.mat'), path{1}))
            continue
        end
        
        result = load(path{1});
        results = [results, result];
    end
end
