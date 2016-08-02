function sum1 = f_eval_err(r, p)
    sum = 0.0;
    global world_size;
    for i = 1:length(p) % calculate mean error
        dx = mod((p(i).x - r.x + (world_size/2.0)), world_size) - (world_size/2.0);
        dy = mod((p(i).y - r.y + (world_size/2.0)), world_size) - (world_size/2.0);
        err = sqrt(dx * dx + dy * dy);
        sum = err + sum;
    end
    sum1 = (sum / length(p)); % Average deviation of robot position from particles
end
   