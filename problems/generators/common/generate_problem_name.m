function name = generate_problem_name(model, problem_options, init_cond_idx)
    name = string(model.model_name);
    idx = string(num2str(init_cond_idx,'%03d'));
    stages = string(num2str(problem_options.N_stages,'%03d'));
    fes = string(num2str(problem_options.N_finite_elements(1),'%03d'));
    n_s = string(num2str(problem_options.n_s,'%d'));
    cc_mode = string(num2str(problem_options.cross_comp_mode ,'%d'));
    vertical = string(num2str(problem_options.lift_complementarities));
    
    switch problem_options.irk_scheme
        case IRKSchemes.RADAU_I
            irk = "RI";
        case IRKSchemes.RADAU_IA
            irk = "RIA";
        case IRKSchemes.RADAU_IIA
            irk = "RIIA";
        case IRKSchemes.GAUSS_LEGENDRE
            irk = "GL";
        case IRKSchemes.LOBATTO_III
            irk = "LIII";
        case IRKSchemes.LOBATTO_IIIA
            irk = "LIIIA";
        case IRKSchemes.LOBATTO_IIIB
            irk = "LIIIB";
        case IRKSchemes.LOBATTO_IIIC
            irk = "LIIIC";
        case IRKSchemes.EXPLICIT_RK
            irk = "ERK";
    end

    dcs = upper(string(problem_options.dcs_mode));

    if problem_options.lift_complementarities
        
    else
        
    end
    
    if problem_options.time_freezing_hysteresis
        source = "HYS";
    elseif problem_options.time_freezing || problem_options.dcs_mode == DcsMode.CLS
        if model.e == 0
            source = "IEC";
        else
            source = "ELC";
        end
    else
        source = "FIL";
    end
    
    name = join([name,idx,stages,fes,n_s,irk,dcs, cc_mode, source, vertical],'_');
end
