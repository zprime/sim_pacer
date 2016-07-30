function makeInfo = rtwmakecfg

if ispc
    makeInfo.linkLibsObjs = { 'winmm.lib' };
end

end