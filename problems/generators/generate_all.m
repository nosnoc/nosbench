function generate_all()
    folder  = '.';
    flist = dir(fullfile(folder, '**/*generator.m'));
    flist.name
    save('flist.mat', 'flist');
    for ii=1:length(flist)
        f = load('flist');
        generator_file = f.flist(ii)
        file = fullfile(generator_file.folder, generator_file.name);
        runfile(file);
        clear all
    end
end

function runfile(file)
    run(file);
end
