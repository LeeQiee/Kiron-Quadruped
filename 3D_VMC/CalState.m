function S=CalState(ctact_singal,tim)

persistent state last_state
% if ~isempty(state)
%     state=4;
% end
% if ~isempty(last_state)
%     last_state=3;
% end

if tim<=1
    state=4;
    last_state=3;
else
    if     last_state==3 && isequal(ctact_singal,[1 1 1 1])
        state=0;
    elseif last_state==3 && isequal(ctact_singal,[0 1 0 1])
        last_state=0;
        
    elseif last_state==0 && isequal(ctact_singal,[1 1 0 1])
        state=1.1;last_state=1;
    elseif last_state==0 && isequal(ctact_singal,[0 1 1 1])
        state=1.2;last_state=1;
    elseif last_state==0 && isequal(ctact_singal,[1 1 1 1])
        state=1;last_state=1;
        
    elseif last_state==1 && isequal(ctact_singal,[1 1 1 1])
        state=2;
    elseif last_state==1 && isequal(ctact_singal,[1 0 1 0])
        last_state=2;
        
    elseif last_state==2 && isequal(ctact_singal,[1 1 1 0])
        state=3.1;last_state=3;
    elseif last_state==2 && isequal(ctact_singal,[1 0 1 1])
        state=3.2;last_state=3;
    elseif last_state==2 && isequal(ctact_singal,[1 1 1 1])
        state=3;last_state=3;
    end
end
S=state;