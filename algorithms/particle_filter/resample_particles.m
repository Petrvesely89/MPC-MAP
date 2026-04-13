function [new_particles] = resample_particles(particles, weights)
%RESAMPLE_PARTICLES Thrun resampling

n = length(weights);
new_particles = zeros(size(particles));

idx = randi(n);
beta = 0;
w_max = max(weights);

for i = 1:n
    
    beta = beta + rand * 2 * w_max;
    
    while beta > weights(idx)
        beta = beta - weights(idx);
        idx = idx + 1;
        
        if idx > n
            idx = 1;
        end
    end
    
    new_particles(i,:) = particles(idx,:);
end

end

