%==============================================================================
% Class Name: OnlineTracking.m
% Author: Arun Niddish Mahendran
% Last modified date: 2024-MM-DD
% Description: This class provides methods for tracking markers on a robot 
%              in real time based on color and size filters, nearest neighbor 
%              algorithm for consistent indexing of markers, reconstruction 
%              of markers when occluded, for calculating rotation and
%              translation of the robot in both global and body frame, data
%              logging, plots of markers overlaid on video at respective
%              frames.
%
% Dependencies:
% 1) createMaskXXXX.m
% 2) rigid_transform_3D.m
% 3) getFrameFromCamera.m
%=============================================================================

classdef OnlineTracking

    % MyClass - Brief description of the class
    % Detailed description of the class and its purpose.

    properties

        % Inputs for number of markers
        number_of_markers;

        % Centroid of each markers
        centroids;

        % Centroid of each marker((n-1)th frame)
        PrevPt;

        % Centroid of each marker(1st frame)
        P0;

        % Centroid of each marker in the current frame (nth frame)
        CurrPt;

        % Centroid of each marker in the current frame
        cent;


        tracking_data_centroid; % Added

        %vread;
        %numberOfFrames;
        
        % Variable holding properies of animated video
        vwrite;

        % overlay_image;
        % overlay;

        % Overall centroid of the robot
        robot_centroid;

        % Current orientation of the robot
        theta_curr;

        % pixel_to_cm;

        % Robot rotated in the current frame
        robot_rotated;

    end

    methods

        function obj = OnlineTracking(params)

            % OnlineTracking - Constructor for the class OnlineTracking
            %
            % Syntax: obj = OnlineTracking(input1)
            %
            % Inputs:
            %    input1 - 'params' a structure containing the parameters for initialization
            %
            % Outputs:
            %    obj - An instance of the class OnlineTracking
            
            obj.number_of_markers = params.number_of_markers;
            obj.PrevPt = [];
            obj.P0 = [];
            obj.CurrPt = [];
            obj.cent = [];
            obj.vwrite = params.vwrite_tracked;
            % obj.overlay_image = params.overlay_img_cut;
            % obj.overlay = params.overlay;
            obj.robot_centroid = [];        
            obj.theta_curr = [];
            obj.robot_rotated = [];
        end

        function [output] = tracking(obj,thisFrame,PrevPt,P0,robot_centroid,theta_curr,k)

            % tracking - Computes a value based on input data
            %
            % Syntax: tracking_data = tracking(obj)
            %
            % Outputs:
            %    output - Data structure containing centroids of each marker on all the frames,
            %             overall centroid of the robot on all the frames,
            %             current orientation of the robot on all the frames,
            %             rotation and translation of the robot in both
            %             global and body frame on all the frames,
            %             and other parameters such as points in previous
            %             frame, initial points, etc.
            

            obj.PrevPt = PrevPt;
            obj.P0 = P0;
            obj.robot_centroid = robot_centroid;
            obj.theta_curr = theta_curr;
            newim = functions.createMaskhdblue(thisFrame);
            newim = bwareaopen(newim,25);
            newim = imfill(newim, 'holes');

            [labeledImage, numberOfRegions] = bwlabel(newim);

            count = 0;
            obj.cent = zeros(numberOfRegions,2);

            stats = regionprops(labeledImage, 'BoundingBox','Centroid','Area','EquivDiameter');
            for rb = 1:numberOfRegions
                count = count + 1;
                obj.cent(count,:) = stats(rb).Centroid;
                obj.cent(count,2) = 1080 - obj.cent(count,2) ;  % Correction for y-axis.
            end

            zc = zeros(size(obj.cent,1),1);
            obj.cent = [obj.cent,zc];

            if k == 1
                obj.P0 = obj.cent;
                output.P0 = obj.P0;
                obj.PrevPt = obj.cent;
                output.PrevPt = obj.PrevPt;
                obj.centroids = data_logging(obj,k);
                theta = zeros(1,9);
                trans = zeros(1,3);
                theta_G = zeros(1,9);
                trans_G = zeros(1,3);
                obj.robot_centroid(k,:) = [mean(obj.PrevPt(:,1)) mean(obj.PrevPt(:,2))];
                output.robot_centroid = obj.robot_centroid;
                obj.robot_rotated = zeros(1,3);
                obj.theta_curr = zeros(1,3);
                output.theta_curr = obj.theta_curr + obj.robot_rotated;
                plot(obj,thisFrame,count,k);
            end

            if k ~= 1

                obj.CurrPt = nearest_neighbor(obj,count);

                [Rot,T] = pose_estimation(obj,obj.CurrPt,obj.PrevPt,k);
                theta(k,:) = reshape(Rot,[1,9]);
                trans(k,:) = T';
                
                [Rot,T] = pose_estimation(obj,obj.P0,obj.CurrPt,k);
                theta_G(k,:) = reshape(Rot,[1,9]);
                trans_G(k,:) = T';

                obj.PrevPt = obj.CurrPt;
                obj.centroids = data_logging(obj,k);

                obj.robot_centroid(k,:) = [mean(obj.PrevPt(:,1)) mean(obj.PrevPt(:,2))];
                output.robot_centroid = obj.robot_centroid;
                obj.robot_rotated = rotm2eul(Rot);
                obj.theta_curr = obj.theta_curr + obj.robot_rotated;
                output.theta_curr = obj.theta_curr;
                plot(obj,thisFrame,count,k);

            end

            output.tracking_data = cat(2,obj.centroids,theta,trans,theta_G,trans_G);
            
        end


        function centroids = nearest_neighbor(obj,count)      %Change the name of the centroid ->

            obj.CurrPt = zeros(obj.number_of_markers,3);
            if(count > obj.number_of_markers)
                for i = 1:obj.number_of_markers
                    for j = 1:count
                        X = [obj.PrevPt(i,:);obj.cent(j,:)];
                        d(j) = pdist(X,'euclidean');
                    end
                    [dmin,ind] = min(d);
                    if(dmin < 15)
                        obj.CurrPt(i,:) = obj.cent(ind,:);
                    end
                end
            end
            if(count <= obj.number_of_markers)
                for i = 1:count
                    for j = 1:obj.number_of_markers
                        X = [obj.cent(i,:);obj.PrevPt(j,:)];
                        d(j) = pdist(X,'euclidean');
                    end
                    [dmin,ind] = min(d);
                    if(dmin < 15)
                        obj.CurrPt(ind,:) = obj.cent(i,:);
                    end
                end
            end

            clear d;
            TF = obj.CurrPt(:,1);  % Writing the 1st column of resrvd
            index = find(TF == 0);  % Finding those rows which is empty
            val = isempty(index);  % Checking whether the index is empty

            if(val == 0)
                centroids = occlusion(obj,index);   
            end
            if(val~=0)
                centroids = obj.CurrPt;       
            end

        end

        function centroids = occlusion(obj,index)   
            newPrevPt = obj.PrevPt;
            newP0 = obj.P0;
            newPrevPt(index(1:size(index,1)),:) = 0;
            newP0(index(1:size(index,1)),:) = 0;
            [Rot,T] = pose_estimation(obj,newPrevPt,obj.CurrPt); % SE2 w.r.t previous frame
            for gg = 1:size(index,1)
                newPt = Rot*(obj.PrevPt(index(gg),:))' + T;
                obj.CurrPt(index(gg),:) = newPt;
            end
            centroids = obj.CurrPt;
        end

        function centroids = data_logging(obj,k)

            for i = 1:obj.number_of_markers
                obj.centroids(k,(3*i)-2:(3*i)) = obj.PrevPt(i,:);
                centroids = obj.centroids;
            end

        end

        function [Rot,T,theta,trans] = pose_estimation(obj,A,B,k)

            [Rot,T] = functions.rigid_transform_3D(A',B');  % SE2 w.r.t previous frame
            %             theta(k,:) = reshape(Rot,[1,9]);   
            %             trans(k,:) = T';

        end

        function plot(obj,thisFrame,count,k)

            figure(10)
            imshow(thisFrame)
            set(gcf, 'Position',  [100, 100, 1080, 1080])
            hold on

            %Plot markers on the robot
            plot(obj.PrevPt(:,1),1080-obj.PrevPt(:,2),'g*','LineWidth',0.5,'MarkerSize',2)

            %Plot centroid of the robot
            plot(obj.robot_centroid(:,1),1080-obj.robot_centroid(:,2),'c*','LineWidth',2.5,'MarkerSize',2)

            % Plot Quiver - Coordinate system of the robot
            rot_val = 0;
            L = 100;
            u = L*cos(obj.theta_curr(1) + rot_val);
            v = L*sin(obj.theta_curr(1) + rot_val);
            u_bar = L*cos(obj.theta_curr(1)+rot_val - pi/2);
            v_bar = L*sin(obj.theta_curr(1)+rot_val - pi/2);
            quiver(obj.robot_centroid(k,1),1080 - obj.robot_centroid(k,2),u,v,'LineWidth',1.7,'Color','b','MaxHeadSize',0.7);
            quiver(obj.robot_centroid(k,1),1080 - obj.robot_centroid(k,2),u_bar,v_bar,'LineWidth',1.7,'Color','g','MaxHeadSize',0.7);

            caption = sprintf('%d blobs found in frame %d', count, k);
            title(caption, 'FontSize', 20);

            %Legends
            % legendnames = {'Robot markers','Robot path traced','Path Planner Path','Goal position'};
            % legend(legendnames);

            axis on;
            hold off
            pframe = getframe(gcf);
            writeVideo(obj.vwrite,pframe);

        end
        %
        %         function plot(obj,thisFrame,count,k)
        % %
        %             figure(10)
        %             imshow(thisFrame)
        %             set(gcf, 'Position',  [100, 100, 1000, 1000])
        %             hold on
        % %             plot(obj.PrevPt(k,1),1080-obj.PrevPt(k,2),'g*','LineWidth',0.5,'MarkerSize',2)
        %             plot(obj.tracking_data_centroid(k,1),1080-obj.tracking_data_centroid(k,2),'g*','LineWidth',0.5,'MarkerSize',2)
        %             caption = sprintf('%d blobs found in frame #%d 0f %d', count, k, obj.numberOfFrames);
        %             title(caption, 'FontSize', 20);
        %             axis on;
        % %             hold off
        %             ovly_img_hdl = imagesc(obj.overlay_image);              % overlay (reference) image
        %             set(ovly_img_hdl, 'AlphaData', 0.4);    % set overlaid image alpha
        %             hold off;
        %             pframe = getframe(gcf);
        %             writeVideo(obj.vwrite,pframe);
        %
        %         end

    end

end
