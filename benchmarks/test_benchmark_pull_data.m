clear all
close all

bench_name = "TEST_BENCH";
BENCH_NAME = [char(bench_name), '_', char(datetime('today','Format','yyyy-MM-dd'))];

dtable = pull_data_from_jobs(BENCH_NAME, @process_data, ["problem_name", "solver_name", "success", "wall_time_total"]);


function row = process_data(stats, options, problem_name)
    row = {};
    row{1} = problem_name;
    row{2} = string(options.solver_name);
    row{3} = stats.success;
    row{4} = stats.wall_time_total;
end
