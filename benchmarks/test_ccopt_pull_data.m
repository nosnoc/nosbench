bench_name = "CCOPT_BENCH";
BENCH_NAME = [char(bench_name), '_', char(datetime('today','Format','yyyy-MM-dd'))];

data_table = pull_data_from_jobs(BENCH_NAME, @process_data, ["problem_name", "solver_name", "success", "wall_time_total"]);

best_time = pivot(data_table, Rows=["problem_name"], Columns=["success"], DataVariable=["wall_time_total"], Method="min");
best_time_dict = dictionary(best_time{:, "problem_name"},best_time{:, "true"});
data_table.ratio_best_time = (data_table.wall_time_total./best_time_dict(data_table.problem_name));


save([char(bench_name) '_results.mat'], 'data_table');


function row = process_data(stats, options, problem_name)
    row = {};
    if metaclass(options) == ?nosnoc.ccopt.Options
        row{1} = problem_name;
        row{2} = string(options.solver_name);
        row{3} = stats.success;
        row{4} = stats.t_wall_total;
    else
        row{1} = problem_name;
        row{2} = string(options.solver_name);
        row{3} = stats.success;
        row{4} = stats.wall_time_total;
    end    
end
