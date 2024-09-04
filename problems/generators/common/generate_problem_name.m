function name = generate_problem_name(model_name, model, problem_options, init_cond_idx)
    name = string(model_name);
    idx = string(num2str(init_cond_idx,'%03d'));
    stages = string(num2str(problem_options.N_stages,'%03d'));
    fes = string(num2str(problem_options.N_finite_elements(1),'%03d'));
    n_s = string(num2str(problem_options.n_s,'%d'));
    cc_mode = string(num2str(problem_options.cross_comp_mode ,'%d'));
    
    switch problem_options.rk_scheme
        case RKSchemes.RADAU_I
            irk = "RI";
        case RKSchemes.RADAU_IA
            irk = "RIA";
        case RKSchemes.RADAU_IIA
            irk = "RIIA";
        case RKSchemes.GAUSS_LEGENDRE
            irk = "GL";
        case RKSchemes.LOBATTO_III
            irk = "LIII";
        case RKSchemes.LOBATTO_IIIA
            irk = "LIIIA";
        case RKSchemes.LOBATTO_IIIB
            irk = "LIIIB";
        case RKSchemes.LOBATTO_IIIC
            irk = "LIIIC";
        case RKSchemes.EXPLICIT_RK
            irk = "ERK";
    end

    dcs = upper(string(problem_options.dcs_mode));

    if problem_options.time_freezing_hysteresis
        source = "HYS";
    elseif problem_options.time_freezing || problem_options.dcs_mode == DcsMode.CLS
        try
            if model.e == 0
                source = "IEC";
            else
                source = "ELC";
            end
        catch
            source = "ELC";
        end
    else
        source = "FIL";
    end
    
    name = join([name,idx,stages,fes,n_s,irk,dcs, cc_mode, source],'_');
end
