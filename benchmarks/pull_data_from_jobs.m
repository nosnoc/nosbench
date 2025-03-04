function dtable = pull_data_from_jobs(bench_name, row_fun, col_names)
    arguments
        bench_name(1,1) string
        row_fun(1,1) function_handle
        col_names(1,:) string
    end
    c = parcluster;
    rows = cell(0,length(col_names));
    jobs = c.findJob(Tag=bench_name);
    for job=jobs
        tasks = findTask(job, @(task) ~(task.hasError)' & [task.StateEnum] == "Finished")';
        for task=tasks
            stats = task.OutputArguments{1};
            options = task.InputArguments{2};
            problem_name = task.InputArguments{3};
            row = row_fun(stats, options, problem_name);

            rows(end+1,:) = row;
        end
    end

    dtable = cell2table(rows, "VariableNames", col_names);
end
