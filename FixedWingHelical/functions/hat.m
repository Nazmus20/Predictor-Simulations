function ehat=hat(e)
ehat=[0 -e(3) e(2);
    e(3) 0 -e(1);
    -e(2) e(1) 0;];
end