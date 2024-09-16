function data_table = process_results(results, stats_fields, solver_opts_fields, problem_opts_fields)
    arguments
        results cell
        stats_fields cell = {'cpu_time_total', 'wall_time_total', 'converged', 'constraint_violation', 'objective(end)', 'complementarity_stats(end)'};
        solver_opts_fields cell = {'solver_name'};
        problem_opts_fields cell = {'N_stages'};
    end

    n_total = numel(results);
    n_sf = numel(stats_fields);
    n_sof = numel(solver_opts_fields);
    n_pof = numel(problem_opts_fields);

    all_structs = {};
    for ii=1:n_total
        tline = struct();
        tline.problem_name = results{ii}.result.problem_name;
        for jj=1:n_sf
            field = stats_fields{jj};
            if endsWith(field, '(end)')
                field = extractBefore(field, '(end)');
                tline.(['stats_' field]) = normalize_string(results{ii}.result.stats.(field)(end));
            else
                tline.(['stats_' field]) = normalize_string(results{ii}.result.stats.(field));
            end
        end

        for jj=1:n_sof
            field = solver_opts_fields{jj};
            val = results{ii}.result.options.(field);
            tline.(['solver_opts_' field]) = normalize_string(val);
        end

        [filepath,~,~] = fileparts(mfilename('fullpath'));
        metadata = jsondecode(fileread([char(filepath) '/../problems/metadata/' char(tline.problem_name), '.json']));
        for jj=1:n_pof
            field = problem_opts_fields{jj};
            val = metadata.opts.(field);
            tline.(['problem_opts_' field]) = normalize_string(val);
        end
        
        all_structs{ii} = tline;
    end
    
    data_table = struct2table([all_structs{:}]);
    best = pivot(data_table, Rows=["problem_name"], Columns=["stats_converged"], DataVariable=["stats_objective"], Method="min");
    best_dict = dictionary(best{:, "problem_name"},best{:, "true"});
    data_table.percent_best = (data_table.stats_objective./best_dict(data_table.problem_name))*100;

    best_time = pivot(data_table, Rows=["problem_name"], Columns=["stats_converged"], DataVariable=["stats_wall_time_total"], Method="min");
    best_time_dict = dictionary(best_time{:, "problem_name"},best_time{:, "true"});
    data_table.ratio_best_time = (data_table.stats_wall_time_total./best_time_dict(data_table.problem_name));
end

function val = normalize_string(val)
    if ischar(val)
        val = string(val);
    end
end
