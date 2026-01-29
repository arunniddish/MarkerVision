%==========================================================================
% Class Name: OfflineTracking.m
% Author: Arun Niddish Mahendran
% Last modified date: 2024-MM-DD
% Description: This class provides methods for tracking markers on a robot
%              based on color and size filters, nearest neighbor algorithm
%              for consistent indexing of markers, reconstruction of
%              markers when occluded, for calculating rotation and
%              translation of the robot in both global and body frame, data
%              logging, plots of markers overlaid on video at respective
%              frames.
%
% Dependencies:
% 1) createMaskXXXX.m
% 2) rigid_transform_3D.m
%==========================================================================

classdef OfflineTracking

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
        
        % Overall centroid of the robot
        robot_centroid;

        % Current orientation of the robot
        theta_curr;

        %overlay;
        
        % Bounding box plotted over the marker
        thisBB;
        
        % Variable holding properties of the video to be tracked
        vread;
        
        % Total number of frames in the video
        numberOfFrames;
        
        % Variable holding properies of animated video 
        vwrite;

        % Robot rotated in the current frame
        robot_rotated;

    end

    methods

        function obj = OfflineTracking(params)

            % OfflineTracking - Constructor for the class OfflineTracking
            %
            % Syntax: obj = OfflineTracking(input1)
            %
            % Inputs:
            %    input1 - 'params' a structure containing the parameters for initialization
            %
            % Outputs:
            %    obj - An instance of the class OfflineTracking

            obj.number_of_markers = params.number_of_markers;
            obj.PrevPt = [];
            obj.P0 = [];
            obj.CurrPt = [];
            obj.cent = [];
            obj.robot_centroid = [];
            obj.theta_curr = [];  % Added now for plotting quiver
            obj.robot_rotated = [];
            obj.vread = params.vread;
            obj.numberOfFrames = obj.vread.NumberOfFrame;
            obj.vwrite = params.vwrite;
            obj.centroids = zeros(obj.numberOfFrames,3*obj.number_of_markers);
            % obj.overlay = params.overlay;
            % obj.thisBB = [];

        end

        function [tracking_data] = tracking(obj)

            % tracking - Computes a value based on input data
            %
            % Syntax: tracking_data = tracking(obj)
            %
            % Outputs:
            %    tracking_data - Centroids of each marker on all the frames

            for k = 1:obj.numberOfFrames
                thisFrame = read(obj.vread,k);
                newim = createMask(thisFrame);
                newim = bwareaopen(newim,25);
                newim = imfill(newim, 'holes');
                [labeledImage, numberOfRegions] = bwlabel(newim);

                count = 0;
                obj.cent = zeros(numberOfRegions,2);

                stats = regionprops(labeledImage, 'BoundingBox','Centroid','Area');
                for rb = 1:numberOfRegions
                    count = count + 1;
                    obj.cent(count,:) = stats(rb).Centroid;
                    obj.cent(count,2) = 1080 - obj.cent(count,2);  % Correction for y-axis.
                    obj.thisBB(count,:) = stats(rb).BoundingBox;
                end

               % obj.robot_centroid(k,:) = [mean(obj.cent(:,1)) 1080-mean(obj.cent(:,2))];   

                zc = zeros(size(obj.cent,1),1);
                obj.cent = [obj.cent,zc];

                if k == 1
                    obj.P0 = obj.cent;
                    obj.PrevPt = obj.cent;
                    obj.centroids = data_logging(obj,k);
                    theta = zeros(1,9);
                    trans = zeros(1,3);
                    theta_G = zeros(1,9);
                    trans_G = zeros(1,3);
                    obj.robot_centroid(k,:) = [mean(obj.PrevPt(:,1)) mean(obj.PrevPt(:,2))];
					obj.robot_rotated = zeros(1,3);
                    obj.theta_curr = zeros(1,3);
                    plot(obj,thisFrame,count,k);
                end

                if k ~= 1

                    obj.CurrPt = nearest_neighbor(obj,count);
                    [Rot,T] = pose_estimation(obj,obj.CurrPt,obj.PrevPt);
                    theta(k,:) = reshape(Rot,[1,9]);
                    trans(k,:) = T';
                    obj.robot_rotated = rotm2eul(Rot);

                    [Rot,T] = pose_estimation(obj,obj.P0,obj.CurrPt);
                    theta_G(k,:) = reshape(Rot,[1,9]);
                    trans_G(k,:) = T';

                    obj.PrevPt = obj.CurrPt;
                    obj.centroids = data_logging(obj,k);
                    obj.robot_centroid(k,:) = [mean(obj.PrevPt(:,1)) mean(obj.PrevPt(:,2))];
					obj.theta_curr = obj.theta_curr + obj.robot_rotated;
                    plot(obj,thisFrame,count,k);

                end

            end

            tracking_data = cat(2,obj.centroids,theta,trans,theta_G,trans_G);
            close(obj.vwrite);
        end

        function centroids = nearest_neighbor(obj,count)      

            % nearest_neighbor - To ensure consistent indexing of the
            %                    markers
            %
            % Syntax: centroids = nearest_neighbor(obj,input1)
            %
            % Inputs:
            %    input1 - 'count' total number of blobs present in the
            %              frame
            %
            % Outputs:
            %    centroids - Sorted [x;y;z] data's of the marker

            obj.CurrPt = zeros(obj.number_of_markers,3);
            if(count > obj.number_of_markers)
                for i = 1:obj.number_of_markers
                    for j = 1:count
                        X = [obj.PrevPt(i,:);obj.cent(j,:)];
                        d(j) = pdist(X,'euclidean');
                    end
                    [dmin,ind] = min(d);
                    if(dmin < 25)
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
                    if(dmin < 25)
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

            % occlusion - Reconstruction of the marker incase of occlusion
            %
            % Syntax: centroids = occlusion(obj,input1)
            %
            % Inputs:
            %    input1 - 'index' the index of the marker that has been
            %              occluded
            %
            % Outputs:
            %    centroids - Final sorted data of the marker [x;y;z]
            %                including the reconstructed marker data

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

            % data_logging - Column to row transition of the data and
            %                sorting according to frame number
            %
            % Syntax: centroids = data_logging(obj,input1)
            %
            % Inputs:
            %    input1 - 'k' current frame number
            %
            % Outputs:
            %    centroids - Computed result

            for i = 1:obj.number_of_markers
                obj.centroids(k,(3*i)-2:(3*i)) = obj.PrevPt(i,:);
                centroids = obj.centroids;
            end

        end

        function [Rot,T,theta,trans] = pose_estimation(obj,A,B)

            % pose_estimation - Computes rotation & translation from the
            %                   centroid data between 2 frames
            %
            % Syntax: centroids = nearest_neighbor(obj,input1,input2,input3)
            %
            % Inputs:
            %    input1 - 'A' centroid data of each marker from the (n-1)th
            %             frame
            %    input2 - 'B' centroid data of each marker from (n)th frame
            %
            % Outputs:
            %    Rot - Rotation matrix (3X3) computed from the centroid  
            %          data of 2 frames 'A' & 'B'
            % 
            %    T   - Translation matrix (3X1) computed from the centroid
            %          data of 2 frames 'A' & 'B'

            [Rot,T] = rigid_transform_3D(A',B');  % SE2 w.r.t previous frame

        end

        function plot(obj,thisFrame,count,k)

            % plot - Plot the centroid and bounding box over the current frame image
            %        and writes the image to .mp4 file to create an
            %        animation movie.
            %
            % Syntax: plot(obj,input1,input2,input3,input4)
            %
            % Inputs:
            %    input1 - 'thisFrame' current frame image
            %    input2 - 'count' total number of blobs present in the
            %              frame
            %    input3 - 'k' current frame index
            %
            % Outputs:
            %    figure

            figure(10)
            imshow(thisFrame)
            set(gcf, 'Position',  [100, 100, 1080, 1080])
            hold on

            %Plot centroid of the markers
            plot(obj.PrevPt(:,1),1080-obj.PrevPt(:,2),'g*','LineWidth',2,'MarkerSize',3)

            %Plot centroid and trajectory of the robot
            plot(obj.robot_centroid(:,1),obj.robot_centroid(:,2),'c*','LineWidth',1,'MarkerSize',1)

            %Plot Bounding Box
            %              for ii = 1:size(obj.thisBB,1)
            %               rectangle('Position', [obj.thisBB(ii,:)],...
            %                     'EdgeColor','y','LineWidth',2 )
            %              end

            % Plot Quiver - Coordinate system of the robot
            rot_val = 0;
            L = 100;
            u = L*cos(obj.theta_curr(1) + rot_val);
            v = L*sin(obj.theta_curr(1) + rot_val);
            u_bar = L*cos(obj.theta_curr(1)+ rot_val - pi/2);
            v_bar = L*sin(obj.theta_curr(1)+ rot_val - pi/2);
            quiver(obj.robot_centroid(k,1),1080 - obj.robot_centroid(k,2),u,v,'LineWidth',1.7,'Color','m','MaxHeadSize',0.7);
            quiver(obj.robot_centroid(k,1),1080 - obj.robot_centroid(k,2),u_bar,v_bar,'LineWidth',1.7,'Color','g','MaxHeadSize',0.7);
            
            orientation_overlay = sprintf('Orientation: %d degree', obj.theta_curr);
            text(50, 50, orientation_overlay , 'Color', 'yellow', 'FontSize', 20);

            caption = sprintf('%d blobs found in frame #%d 0f %d', count, k, obj.numberOfFrames);
            title(caption, 'FontSize', 20);
            axis on;
            hold off
            pframe = getframe(gcf);
            writeVideo(obj.vwrite,pframe);

        end

    end

end
