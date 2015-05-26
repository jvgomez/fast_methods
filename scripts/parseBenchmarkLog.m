function bm = parseBenchmarkLog (path_to_file)
    %% Opening file.
    txt = fileread(path_to_file);
    txt = regexprep(fileread(path_to_file), '\s+', '\t'); % Spaces (if any) to tabs.
    txt = regexp(txt, '[\t\n]', 'split');

    %% Parsing header.
    bm.name = txt{1};
    bm.nruns = str2double(txt{2});
    bm.ndims = str2double(txt{3});
    bm.dimsize = zeros(bm.ndims,1);
    for i = 1:bm.ndims
        bm.dimsize(i) = str2double(txt{i+3});
    end

    nstartpoints = str2double(txt{4+bm.ndims});
    bm.startpoints = zeros(nstartpoints,1);
    for i = 1:nstartpoints
        bm.startpoints(i) = str2double(txt{4+bm.ndims+i});
    end

    bm.goalpoint = str2double(txt{5+bm.ndims+nstartpoints});

    hs = 5+bm.ndims+nstartpoints; % Header's length

    %% Parsing experiments. Might be a bit redundant.
    bm.nexp = (length(txt)-hs)/3;
    id = zeros(bm.nexp,1);
    idstr = cell(bm.nexp,1);
    solvers = cell(bm.nexp/bm.nruns,1);
    times = zeros(bm.nexp,1);
    for i = 1:bm.nexp
        idx = hs+(i-1)*3 + 1;
        idstr{i} = txt{idx};
        id(i) = str2double(idstr(i));
        solvers{i} = txt{idx+1};
        times(i) = str2double(txt{idx+2});
    end

    bm.exp = cell(bm.nexp/bm.nruns,2);
    for i = 1:bm.nexp/bm.nruns
        bm.exp{i,1} = solvers{(i-1)*bm.nruns+1};
        bm.exp{i,2} = times((i-1)*bm.nruns+1:i*bm.nruns);
    end

