function zd_dothat =zd_dot(t, freq,A)
    zd_dothat = A*cos(freq*t)*freq;
end