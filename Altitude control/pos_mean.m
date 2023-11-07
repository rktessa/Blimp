function [pos_list, flag] = pos_mean(pos_list, pos_meas)
    
    mean_pos = mean(pos_list);
    flag = 0;
    
    if (pos_meas <= (mean_pos + 0.2)) && (pos_meas >= (mean_pos - 0.2))
        pos_list(end+1) = pos_meas;
        pos_list(1) = [];
    else
        flag = 1;
    end
end