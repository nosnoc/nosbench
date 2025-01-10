function generate_all()
    folder  = '.';
    flist = dir(fullfile(folder, '**/*generator.m'));
    flist.name;
    orig_dir = pwd;
    c = parcluster;
    for ii=1:length(flist)
        generator_file = flist(ii);
        file = fullfile(generator_file.folder, generator_file.name);
        [path, name, ~] = fileparts(file);
        cd(path);
        path
        name
        batch(name, 'CaptureDiary', true);
        c.Jobs
    end
    cd(orig_dir);
end

function runfile(file)
    run(file);
end
