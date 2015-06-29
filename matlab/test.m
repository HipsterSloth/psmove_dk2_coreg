%% Load the data

poses = readtable(fullfile(pwd,'output.txt'));
% poses.dk2_px = poses.dk2_px / 100;
% poses.dk2_py = poses.dk2_py / 100;
% poses.dk2_pz = poses.dk2_pz / 100;
camera_pose = csvread(fullfile(pwd,'output_camerapose.txt'));
% camera_pose(1:3) = camera_pose(1:3)/100;
camera_4x4 = makehgtform('translate', camera_pose(1:3),...
    'axisrotate', camera_pose(4:6), camera_pose(7));
camera_invxform = [camera_4x4(1:3,1:3)', -camera_4x4(1:3,1:3)'*camera_4x4(1:3,4); 0 0 0 1];

n = size(poses, 1);

%% Calculate the transforms
A = nan(3*n, 15);
b = nan(3*n, 1);
for p = 1:n
    h = makehgtform(...
        'translate', [poses.dk2_px(p), poses.dk2_py(p), poses.dk2_pz(p)],...
        'axisrotate', [poses.dk2_ox(p), poses.dk2_oy(p), poses.dk2_oz(p)], poses.dk2_ow(p));
    h = camera_invxform * h;
    RMi = h(1:3, 1:3)';
    
    Ti = makehgtform(...
        'translate', [poses.psm_px(p), poses.psm_py(p), poses.psm_pz(p)],...
        'axisrotate', [poses.psm_ox(p), poses.psm_oy(p), poses.psm_oz(p)], poses.psm_ow(p));
    
    A((p-1)*3 + (1:3), :) = [RMi * Ti(1,4), RMi * Ti(2,4), RMi * Ti(3,4), RMi, -eye(3)];
    b((p-1)*3 + (1:3)) = RMi * h(1:3, 4);
end

x = A\b;

globalxfm = reshape(x(1:12), 3, 4);
localxfm = [1 0 0 x(12); 0 1 0 x(13); 0 0 1 x(14); 0 0 0 1];

clear A b p h RMi Ti x

%% Plot the result
dk2_xyz = [poses.dk2_px, poses.dk2_py, poses.dk2_pz]';
%dk2_xyz = camera_invxform(1:3,:) * [dk2_xyz; ones(1, n)];
psm_xyz = [poses.psm_px, poses.psm_py,  poses.psm_pz]';
lims = [min([psm_xyz'; dk2_xyz']); max([psm_xyz'; dk2_xyz'])];
subplot(2,2,1)
plot3(dk2_xyz(1,:), dk2_xyz(3,:), dk2_xyz(2,:), 'k',...
    psm_xyz(1,:), psm_xyz(3,:), psm_xyz(2,:), 'm',...
    'LineWidth', 3)
xlabel('X')
ylabel('Z')
zlabel('Y')
xlim([lims(1,1) lims(2,1)]);
ylim([lims(1,3) lims(2,3)]);
zlim([lims(1,2) lims(2,2)]);
legend('DK2', 'PSMove', 'Location', 'North')
legend('boxoff')
set(gca, 'FontSize', 14)
set(gca, 'Color', 'none')
%set(gca, 'CameraViewAngle', 8.7)

subplot(2,2,2)
hist(sqrt(sum((dk2_xyz - psm_xyz).^2)));
xlabel('Eucl. Distance (cm)')
ylabel('Count')
set(gca, 'Color', 'none')
set(gca, 'FontSize', 14)
title('Before Coreg.')
box off

tpsm_xyz = camera_4x4 * [globalxfm; 0 0 0 1] * [psm_xyz; ones(1, n)];
tpsm_xyz = tpsm_xyz(1:3, :);
subplot(2,2,3)
plot3(dk2_xyz(1,:), dk2_xyz(3,:), dk2_xyz(2,:), 'k',...
    tpsm_xyz(1,:), tpsm_xyz(3,:), tpsm_xyz(2,:), 'm',...
    'LineWidth', 3)
xlabel('X')
ylabel('Z')
zlabel('Y')
lims = [min([tpsm_xyz'; dk2_xyz']); max([tpsm_xyz'; dk2_xyz'])];
xlim([lims(1,1) lims(2,1)]);
ylim([lims(1,3) lims(2,3)]);
zlim([lims(1,2) lims(2,2)]);
set(gca, 'FontSize', 14)
set(gca, 'Color', 'none')
%set(gca, 'CameraViewAngle', 8.7)

subplot(2,2,4)
hist(sqrt(sum((dk2_xyz - tpsm_xyz(1:3,:)).^2)));
xlabel('Eucl. Distance (cm)')
ylabel('Count')
title('After Coreg.')
set(gca, 'Color', 'none')
set(gca, 'FontSize', 14)
box off