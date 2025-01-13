close all
clear all
problems = get_problems_from_file('../unlifted_problems');
parfor ii=1:length(problems)
    problem = problems(ii);
    outfile = ['../casadi/', problem.name, '.json'];
    mpcc = NosnocMPCC(problem.problem_options, problem.model);
    json = jsonencode(mpcc.to_serialized_casadi_mpcc, "ConvertInfAndNaN",false, "PrettyPrint", true);
    fid = fopen(outfile, 'w');
    fprintf(fid, json);
    fclose(fid);
end
