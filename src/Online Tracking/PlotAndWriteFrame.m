function PlotAndWriteFrame()

if Q.QueueLength > 0

	plot_datas = poll(Q); 
	figure(10)
    imshow(thisFrame)
    set(gcf, 'Position',  [100, 100, 1000, 1000])
    hold on
            
    %Plot markers on the robot
    plot(obj.PrevPt(:,1),1080-obj.PrevPt(:,2),'g*','LineWidth',0.5,'MarkerSize',2)
%   plot(obj.tracking_data_centroids(k,1),1080-obj.tracking_data_centroid(k,2),'g*','LineWidth',0.5,'MarkerSize',2)
     
    %Plot centroid of the robot
    plot(obj.robo_centroid(:,1),1080-obj.robo_centroid(:,2),'c*','LineWidth',2.5,'MarkerSize',2)
            
    %Plot path planner trajectory
%    plot(obj.trajectory_position(1,:)*obj.pixel_to_cm,1080-(obj.trajectory_position(2,:)*obj.pixel_to_cm),'-r','LineWidth',2);
    %plot(trajectory_position(1,:)*obj.pixel_to_cm,1080-(trajectory_position(2,:)*obj.pixel_to_cm),'-r','LineWidth',2);
            
    %Plot quiver
%    plot(obj.robo_centroid(k,1),1080-obj.robo_centroid(k,2),40*cos(obj.robo_rotated),10*sin(obj.robo_rotated),'LineWidth',2,'Color',"#D95319");
            
            
    % Bounding box
    %for ii = 1:size(obj.obstaclesBB,1)
         %h = rectangle('Position',obj.obstaclesBB(ii,:),'EdgeColor','y','LineWidth',3);
    %end
            
    % Target/Goal position
    %plot(obj.target_position(:,1),1080-obj.target_position(:,2),'r*','LineWidth',0.7,'MarkerSize',5)
    %viscircles([obj.target_position(:,1) 1080-obj.target_position(:,2)],obj.stop_radius,'LineStyle','--');

    caption = sprintf('%d blobs found in frame %d', count, k);
    title(caption, 'FontSize', 20);
            
    %Legends
    %legendnames = {'Robot markers','Robot path traced','Path Planner Path','Goal position'};
    %legend(legendnames);
            
    axis on;
    hold off
    pframe = getframe(gcf);
    writeVideo(obj.vwrite,pframe);
	
end

end	

