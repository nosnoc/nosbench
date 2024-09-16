function stats = solve_with_homotopy_solver(mpcc, options)
    mpcc.create_solver(options);
    stats = mpcc.solve();
end
