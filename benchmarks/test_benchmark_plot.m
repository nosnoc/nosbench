function test_benchmark_plot(result_dir, cutoff, settings)
    arguments
        result_dir string
        cutoff double
        settings.absolute = true
        settings.relative = true
        settings.save_plot = false
    end
    color_order = [0 0.4470 0.7410;
        0.8500 0.3250 0.0980;
        0.9290 0.6940 0.1250;
        0.4940 0.1840 0.5560;
        0.4660 0.6740 0.1880;
        0.3010 0.7450 0.9330;
        0.6350 0.0780 0.1840];
    line_style_cycling_method = 'aftercolor';  % 'withcolor'; % 'beforecolor'; ' aftercolor'
    line_style_order = ["-";"--";":";"-.";"-o";"-x"];  % ["-.";"-x";"-o";"-s"];
    fontsize = 12;
    
    results = load_results(result_dir);
    data_table = process_results(results);

    converged_table = data_table(data_table{:,"stats_converged"} & ((isnan(data_table{:,"percent_best"}) | data_table{:,"percent_best"} < cutoff) | data_table{:, "problem_opts_N_stages"} == 1),:);

    wall_max = max(converged_table{:, "stats_wall_time_total"});
    cpu_max = max(converged_table{:, "stats_cpu_time_total"});
    counts_table = pivot(data_table, Rows=["solver_opts_solver_name"]);

    % make figures    
    if settings.absolute
        handle_fig_absolute = figure('Position', [100, 100, 600, 420]);
        xlabel('Wall Time (s)')
        ylabel('fraction solved')

        for ii=1:height(counts_table)
            n_res = counts_table{ii, 'count'};
            step_size = 1/n_res;
            solver_name = counts_table.solver_opts_solver_name(ii);
            hold off;
            figure(handle_fig_absolute);
            hold on;
            times = sort(converged_table.stats_cpu_time_total(converged_table.solver_opts_solver_name == solver_name));
            levels = step_size*(1:length(times));
            label = [char(solver_name)];
            stairs([0,times',max(data_table.stats_cpu_time_total)*1.1], [0,levels,levels(end)], 'LineWidth', 2.0, 'DisplayName', label);
        end
        hold off
        figure(handle_fig_absolute);
        set(gca,'xscale','log');
        set(gca,'fontsize', fontsize);
        ax = gca;
        ax.ColorOrder= color_order;
        ax.LineStyleOrder = line_style_order;
        ylim([0, 1.0]);
        ax.LineStyleCyclingMethod = line_style_cycling_method;
        hold off;
        legend('Location', 'southeast')
        % grid on;
        if settings.save_plot
            exportgraphics(gca, ['test_bench_absolute.pdf']);
        end
    end

    if settings.relative
        % Relative plots
        handle_fig_relative = figure('Position', [100, 100, 600, 420]);
        xlabel('$2^{x}$ times best')
        ylabel('fraction solved')
        for ii=1:height(counts_table)
            n_res = counts_table{ii, 'count'};
            step_size = 1/n_res;
            solver_name = counts_table.solver_opts_solver_name(ii);
            hold off;
            figure(handle_fig_relative);
            hold on;
            ratios = sort(converged_table.ratio_best_time(converged_table.solver_opts_solver_name == solver_name));
            levels = step_size*(1:length(ratios));
            label = [char(solver_name)];
            stairs(log2([0,ratios',2^15]), [0,levels,levels(end)], 'LineWidth', 2.0, 'DisplayName', label);
        end
        hold off
        figure(handle_fig_relative);
        set(gca,'fontsize', fontsize);
        ax = gca;
        ax.ColorOrder= color_order;
        ax.LineStyleOrder = line_style_order;
        ylim([0, 1.0]);
        ax.LineStyleCyclingMethod = line_style_cycling_method;
        hold off;
        % grid on;
        legend('Location', 'southeast');
        if settings.save_plot
            exportgraphics(gca, [filename '_relative.pdf']);
        end
    end
end
