function [new_path] = smooth_path(old_path)
%SMOOTH_PATH Summary of this function goes here

alfa = 0.25;
    beta = 0.5;
    iterations = 100;

    new_path = old_path;

    for iter = 1:iterations
        for i = 2:size(old_path,1)-1

            new_path(i,:) = new_path(i,:) ...
                + alfa * (old_path(i,:) - new_path(i,:)) ...
                + beta * (new_path(i-1,:) + new_path(i+1,:) - 2 * new_path(i,:));

        end
    end

end

