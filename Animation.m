function [] = Animation(pos,angles,param)
%ANIMATION Make Plot animation by selecting a few points
for i=1:(size(pos,1)-1)
    trajectory(:,:,i)= CalcTrajectory(pos(i,:),pos(i+1,:),2);
    theta(:,:,i)= [angles(i,1) angles(i+1,1); angles(i,2) angles(i+1,2); angles(i,3) angles(i+1,3)];
end
N=size(theta,2);
%simulation step size
dt=0.05;
count=1;
for i=1:size(trajectory,3)
    traj=trajectory(:,:,i);
    ang = theta(:,:,i);
    for j=1:N 
        tic
        PlotPosition(traj(:,j),ang(:,j),param);
        toc
        pause(dt);
        Blah(count) = getframe(gcf);
        drawnow
        count=count+1;
    end
    
        
end
video=VideoWriter('myvideo.avi','Uncompressed AVI')
open(video)
writeVideo(video,Blah)
close(video)
end




