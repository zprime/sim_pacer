function make_sfun_sim_pacer
if ispc
    mex('-lwinmm','sfun_sim_pacer.c');
else
    mex('sfun_sim_pacer.c');
end