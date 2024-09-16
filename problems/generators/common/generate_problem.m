function discrete_time_problem = generate_problem(name ,model, opts)
    % Always process model and options
    opts.preprocess();
    model.verify_and_backfill(opts);

    if class(model) == "nosnoc.model.Cls" && opts.time_freezing
        model = nosnoc.time_freezing.reformulation(model, opts);
        model.verify_and_backfill(opts);
        model = model;
    end

    % Run pipeline
    switch class(model)
      case "nosnoc.model.Pss"
        if opts.dcs_mode == DcsMode.Stewart
            dcs = nosnoc.dcs.Stewart(model);
            dcs.generate_variables(opts);
            dcs.generate_equations(opts);
            discrete_time_problem = nosnoc.discrete_time_problem.Stewart(dcs, opts);
            discrete_time_problem.populate_problem();
        elseif opts.dcs_mode == DcsMode.Heaviside
            dcs = nosnoc.dcs.Heaviside(model);
            dcs.generate_variables(opts);
            dcs.generate_equations(opts);
            discrete_time_problem = nosnoc.discrete_time_problem.Heaviside(dcs, opts);
            discrete_time_problem.populate_problem();
        else
            error("nosnoc: PSS models can only be reformulated using the Stewart or Heaviside Step reformulations.")
        end
      case "nosnoc.model.Heaviside"
        dcs = nosnoc.dcs.Heaviside(model);
        dcs.generate_variables(opts);
        dcs.generate_equations(opts);
        discrete_time_problem = nosnoc.discrete_time_problem.Heaviside(dcs, opts);
        discrete_time_problem.populate_problem();
      case "nosnoc.model.Cls"
        if ~opts.use_fesd
            error("nosnoc: The FESD-J reformulation only makes sense with use_fesd=true.")
        end
        dcs = nosnoc.dcs.Cls(model);
        dcs.generate_variables(opts);
        dcs.generate_equations(opts);
        discrete_time_problem = nosnoc.discrete_time_problem.Cls(dcs, opts);
        discrete_time_problem.populate_problem();
      case "nosnoc.model.Pds"
        if ~opts.right_boundary_point_explicit
            error("nosnoc: You are using an rk scheme with its right boundary point (c_n) not equal to one. Please choose another scheme e.g. RADAU_IIA.")
        end
        if opts.rk_representation == RKRepresentation.differential
            error("nosnoc: Differential representation without lifting is unsupported for gradient complementarity systems. Use integral or lifted differential representation.")
        end
        dcs = nosnoc.dcs.Gcs(model);
        dcs.generate_variables(opts);
        dcs.generate_equations(opts);
        discrete_time_problem = nosnoc.discrete_time_problem.Gcs(dcs, opts);
        discrete_time_problem.populate_problem();
      otherwise
        error("nosnoc: Unknown model type.")
    end

    % Do sorting
    discrete_time_problem.w.sort_by_index();
    discrete_time_problem.g.sort_by_index();

    % create jsons
    json = jsonencode(discrete_time_problem, "ConvertInfAndNaN", false, "PrettyPrint", true);
    casadi_json = discrete_time_problem.to_casadi_json();
    metadata.dims = model.dims;
    metadata.opts = opts;
    metadata_json = jsonencode(metadata, "ConvertInfAndNaN", false, "PrettyPrint", true);

    % write vdx formatted json
    fid = fopen(['../../vdx/', char(name), '.json'], 'w');
    fprintf(fid, '%s', json);
    fclose(fid);

    % write casadi struct formatted json
    fid = fopen(['../../casadi/', char(name), '.json'], 'w');
    fprintf(fid, '%s', casadi_json);
    fclose(fid);

    % write metadata (dims, opts)
    fid = fopen(['../../metadata/', char(name), '.json'], 'w');
    fprintf(fid, '%s', metadata_json);
    fclose(fid);
end
