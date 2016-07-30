function make_sim_pacer
if ispc
    mex('-lwinmm','sim_pacer.c');
else
    mex('sim_pacer.c');
end