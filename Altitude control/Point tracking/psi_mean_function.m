function [psi] = psi_mean_function(psi_list, psi_meas)
    
    sum_psi = 0;
    
    for i = 1:length(psi_list)
        sum_psi = sum_psi + (psi_list(i) - psi_meas)^2;
    end

    if sum_psi/length(psi_list) >= 50
        psi = psi_meas;
    else
        psi = sum(psi_list)/length(psi_list);
    end
end


    