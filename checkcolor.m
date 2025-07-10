function colout=checkcolor(colin)
    err=false;
    if ischar(colin)
        switch colin
            case {'red','r'}
                colout=[1 0 0];
            case {'green','g'}
                colout=[0 1 0];
            case {'blue','b'}
                colout=[0 0 1];
            case {'cyan','c'}
                colout=[0 1 1];
            case {'magenta','m'}
                colout=[1 0 1];
            case {'yellow','y'}
                colout=[1 1 0];
            case {'black','k'}
                colout=[0 0 0];
            case {'white','w'}
                colout=[1 1 1];
            otherwise
                err=true;
        end
    elseif isa(colin,'double') && length(colin)==3
        if isempty(find(colin<0,1)) && isempty(find(colin>1,1))
            colout=colin;
        else
            err=true;
        end
    else
        err=true;
    end
    if err
        if isa(colin,'double')
            error(['Invalid color input: ' num2str(colin)])
        else
            error(['Invalid color input: ' colin])
        end
    end
end