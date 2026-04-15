function [public_vars] = student_workspace(read_only_vars,public_vars)
%STUDENT_WORKSPACE Summary of this function goes here

% 8. Perform initialization procedure
if (read_only_vars.counter == 1)
          
    public_vars = init_particle_filter(read_only_vars, public_vars);
    public_vars = init_kalman_filter(read_only_vars, public_vars);

end

% 9. Update particle filter
public_vars.particles = update_particle_filter(read_only_vars, public_vars);

% 10. Update Kalman filter
[public_vars.mu, public_vars.sigma] = update_kalman_filter(read_only_vars, public_vars);

% 11. Estimate current robot position
if (read_only_vars.counter == 1)
    public_vars.estimated_poses = [];
end

public_vars.estimated_pose = estimate_pose(public_vars);


% 12. Path planning
if read_only_vars.counter<2
    public_vars.path = plan_path(read_only_vars, public_vars);
end


%% ----Week2---Task2: sběr dat---- %
% - For indoor map and LiDAR senzor - %
% if (read_only_vars.counter == 1)
%     public_vars.lidar_data = [];
% end
% 
% if (read_only_vars.counter <= 100)
%     public_vars.lidar_data = [public_vars.lidar_data; read_only_vars.lidar_distances];
% end

% - For outdoor map and GNSS senzor - %
% if (read_only_vars.counter == 1)
%      public_vars.gnss_data = [];
% end
% 
% if (read_only_vars.counter <= 100)
%     public_vars.gnss_data = [public_vars.gnss_data; read_only_vars.gnss_position];
% end


% 13. Plan next motion command
public_vars = plan_motion(read_only_vars, public_vars);
end

