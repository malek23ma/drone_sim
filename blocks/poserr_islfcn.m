function e_pos = poserr_islfcn(in)
%POSERR_ISLFCN  Position error for the integral term.
%   Input  in = [ xhat(1:12) ; ref(13:22) ]   (the cMux output)
%   Output e_pos = pos_ref - pos_est   (3x1) -> integrated externally into e_int.
    e_pos = in(13:15) - in(1:3);
end
