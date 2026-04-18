function generate_all()
    folder  = '.';
    flist = dir(fullfile(folder, '**/*generator.m'));
    flist.name;
    orig_dir = pwd;
    c = parcluster;
    !mkdir ../vdx
    !mkdir ../casadi
    !mkdir ../metadata
    for ii=1:length(flist)
        cd(flist(ii).folder);
        [~,name,~] = fileparts(flist(ii).name);
        job = batch(name, 'CaptureDiary', true, 'AutoAttachFiles', false);
        jobs(ii) = job;
        %msg = char(formattedDisplayText(jobs, 'SuppressMarkup', true));
        %update_msg(msg);

        cd(orig_dir);
    end

    while true
        msg = char(formattedDisplayText(jobs, 'SuppressMarkup', true));
        update_msg(msg);
        all_done = true;
        for job=jobs
            all_done = all_done & strcmp(job.State, 'finished');
        end
        if all_done
            break
        end
        pause(10);
    end

    % tarball and compress using xz
    % Broken by R2025a
    %!tar -c -v -I 'xz -9 -T0' -f ../casadi.tar.xz ../casadi/* ../metadata/*
    %!tar -c -v -I 'xz -9 -T0' -f ../vdx.tar.xz ../vdx/* ../metadata/*
    %!rm -rf ../vdx/*
    %!rm -rf ../casadi/*
    %!rm -rf ../metadata/*
end

function update_msg(msg)
    ASCII_BKSP_CHAR = 8;
    persistent prev_len;
    if isempty(prev_len) 
        prev_len = 0;
    end
    
    %disp([ char(repmat(ASCII_BKSP_CHAR,1,prev_len)) msg]);
    disp(msg);
    prev_len = numel(msg)+1;
end
