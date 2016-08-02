function est_pos = f_get_position(p)
    x = 0.0;
    y = 0.0;
    orientation = 0.0;
    for i = 1:(length(p))
        x = x + p(i).x;
        y = y + p(i).y;
        orientation = orientation + ((mod((p(i).orientation - p(1).orientation + pi), (2.0 * pi))) ...
                        + p(1).orientation - pi);
    end
    est_pos = [x / length(p), y / length(p), orientation / length(p)];

end