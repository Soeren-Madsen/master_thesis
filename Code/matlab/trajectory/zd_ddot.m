function zd_ddothat =zd_ddot(t, freq, A)
    zd_ddothat = -A*sin(freq*t)*freq^2;
end