view_animation = true;
pk = all_states;

pmin = [min(pk(1, :, :), [], "all"), min(pk(2, :, :), [], "all")];
pmax = [max(pk(1, :, :), [], "all"), max(pk(2, :, :), [], "all")];

T = 0.01*(size(pk,3)-1);
t = 0:0.01:T;

%% Initialize video
myVideo = VideoWriter('traj'); %open video file
myVideo.FrameRate = 10;  %can adjust this, 5 - 10 works well for me
open(myVideo)

%% Animation of transition
figure(3)
colors = distinguishable_colors(N);
set(gcf, 'Position', get(0, 'Screensize'));
set(gcf,'currentchar',' ')

idx = 1;
offset = 0;

for k = 1:size(pk, 2)
    if k == sum(all_lengths(1:idx))
        offset = k-1;
        clf
        idx = idx + 1;
    end
    for i = 1:N
        hold on;
        grid on;

        xlim([pmin(1),pmax(1)])
        ylim([pmin(2),pmax(2)])

        plot3(pk(1,offset+1:k,i),pk(2,offset+1:k,i),pk(3,offset+1:k,i),'o',...
            'LineWidth',2,'Color',colors(i,:));
        plot3(goals(idx,1,i), goals(idx,2,i), goals(idx,3,i),'x',...
              'LineWidth',2,'Color',colors(i,:));
    end
    drawnow
    frame = getframe(gcf); %get frame
    writeVideo(myVideo, frame);
end
clf
pause(0.1)
close(myVideo)
