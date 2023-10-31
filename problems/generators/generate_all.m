function generate_all()
    folder  = '.';
    flist = dir(fullfile(folder, '**/*generator.m'));
    flist.name
    for ii=1:length(flist)
        generator_file = flist(ii)
        save('flist.mat', 'flist');
        run(fullfile(generator_file.folder, generator_file.name));
        clear all
        load('flist');
    end
end
