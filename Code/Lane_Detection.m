close all
clear all

% Display Images and Graphs
display_info = true;

% Iterate Once for dev
iter_once = true;

% Just View as image Video
just_view = false;


% Initialize Image array size
Lane = zeros(720,1280,3);

%Create Video Reader Object
vidObj = VideoReader('/home/vendetta/Documents/ENPM673_HW1/project_video.mp4');

% Set Frame grab start time
vidObj.CurrentTime = 20;

% Keep count of total frames processed
frame_count = 1;

% Keep track of each video section
section=1;

gradient_old = 0;

gradient_array_l = [0 0 0 0 0];
gradient_array_r = [0 0 0 0 0];

% Main Loop to process each frame
while hasFrame(vidObj)
    
    % Read each frame from the video reader object
    vidFrame = readFrame(vidObj);
    
    if display_info==true
        figure
        imshow(vidFrame);
    end

    % Convert Frame RGB to grayscale
    vidFramegray = rgb2gray(vidFrame);
    
    if display_info==true
        figure
        imshow(vidFramegray);
    end
    
    % Edge detection filter
    h = fspecial('sobel');
    
    % Detect edges and convert to binary image
    sobel_edge = im2bw(imfilter(vidFramegray,h),0.1);

%    sobel_edge = edge(vidFramegray,'canny');
    
    % Extract white and yellow objects (lanes) and binarize
    max_img = max(vidFrame(:,:,1),vidFrame(:,:,2));
    color_mask = im2bw(max_img,0.65);
    
    % Read trapezoid lane mask
    area_mask = im2bw(imread('/home/vendetta/Documents/ENPM673_HW1/bw_mask_trap2.jpg'));
    
    % mask edges detected with white and yellow objects to isolate lanes
    edgebw = color_mask.*area_mask.*sobel_edge;
    
    if display_info==true
        figure
        imshow(edgebw);
        figure
        imshowpair(sobel_edge,color_mask,'montage');
    end
    
    % Clear irrelevant upper and lower portions of the image
    edgebw(1:445,:,:)=0;

    if display_info==true
        figure
        imshow(edgebw);
    end

    % Clean small dots and lines
%     edgebw=bwmorph(edgebw,'thin');
%     edgebw=bwmorph(edgebw,'spur');
     edgebw=bwmorph(edgebw,'clean');
     edgebw=bwareaopen(edgebw,3);
    
    if display_info==true
        imshow(edgebw);
        figure
    end

    % Perform Hough transform
    [H,T,R] = hough(edgebw,'RhoResolution',1,'ThetaResolution',1);
    
    % Display the Hough transform data
    if display_info == true
        hough_plot = figure;
        imshow(imadjust(rescale(H)),'XData',T,'YData',R,...
              'InitialMagnification','fit');
        xlabel('\theta'), ylabel('\rho');
        axis on, axis normal, hold on;
        colormap(gca,hot);
    end
   
   %% Right Lanes
   
    % Consecutive Thetas to consider in HT
    theta_range = 30;
    
    % locate desired theta index in HT data
    x =  find(T==-82);
    
    % Create HT search line x points
    x = [ x : x+theta_range ]';

    % locate desired rho in hough transform data
    rho = find(R == -408);
    
    % gradient of the search line in HT
    gradient_search=16;
    
    % rho search window for a given theta in HT 
    local_window=100;
    rho_search_window=100/2;
    
    % Create HT search line y points
    y = [rho:gradient_search:rho+theta_range*gradient_search]';
    
    % Initial Search line points
    main_xr=x;
    main_yr=y;
    
    % Threshold to take hough value into consideration
    threshold=20;
    
    % Locally search for dense points near the initial seach line
    n = size(x,1);
    j=1;
    i=1;
    while j<=n
        mid_ind=find(H(y(i)-rho_search_window:y(i)+rho_search_window,x(i))>=threshold);%H(y(i),x(i)))

        x=[x(1:i,1); ones(size(mid_ind))*x(i,1); x(i+1:end,1)];
        y=[y(1:i,1); mid_ind+y(i,1)-rho_search_window-1; y(i+1:end,1)];
        old_i=i;
        i=size(x(1:i),1)+ size(mid_ind,1)+1;
        if H(y(old_i),x(old_i))<threshold
            x=[x(1:old_i-1);x(old_i+1:end,1)];
            y=[y(1:old_i-1);y(old_i+1:end,1)];
            i=i-1;
        end
        j=j+1;
        if i>size((x),1)
            break;
        end
    end
    
    % Hough Points for the right Lane
    P = [y x];    
    
    % Concatenate search line for both lanes 
    P_main = [main_xr main_yr];
  
    % Display inital search line(green) and collected points(blue)
    if display_info == 1 
        
        figure(hough_plot) ;
        plot(T(:,P(:,2)),R(:,P(:,1)),'s','color','blue');
        plot(T(:,P_main(:,1)),R(:,P_main(:,2)),'s','color','green');
        hold on
        
    end
    
    % check if we have any collected points and only then proceed to create
    % lines
    if size(P,1)>0
        % Create lines structure from selected hough points to image space
        lines = houghlines(edgebw,T,R,P,'FillGap',150,'MinLength',7);

        max_len = 0;
        
        
        if exist('lines','var')==1
            
            if display_info == 1 
                lines_plot = figure;
                imshow(edgebw)
                hold on
            end
            
            for k = 1:length(lines)
               xy = [lines(k).point1; lines(k).point2];

               if display_info == 1 
                    

                   % Plot corresponding Lines
                   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

                   % Plot beginnings and ends of lines
                   plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
                   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');              
               end

               % Determine the endpoints of the longest line segment
               len = norm(lines(k).point1 - lines(k).point2);
               if ( len > max_len)
                  max_len = len;
                  xy_long_r = xy;

               end
            end

     %%     

            points1_re=zeros(1,2);
            points2_re=zeros(1,2);
            lines_cell = struct2cell(lines);
            points1 = cell2mat(lines_cell(1,1,:));
            points2 = cell2mat(lines_cell(2,1,:));
            if size(points1,1) ~= 0

                for ind = 1:size(points1,3)
                    points1_re=[points1_re ; points1(1,1,ind) points1(1,2,ind)];
                    points2_re=[points2_re ; points2(1,1,ind) points2(1,2,ind)];
                end

                y_max = 681;
                y_min = 451;

                gradient = (xy_long_r(2,2)-xy_long_r(1,2))/(xy_long_r(2,1)-xy_long_r(1,1));

                if frame_count == 1
                    gradient_array_r = ones(1,10)*gradient;
                end
                gradient_array_r = [gradient_array_r(2:end) gradient];
                gradient_eff = sum(gradient_array_r)/size(gradient_array_r,2);
                gradient_old_r = gradient_eff;

                c = (xy_long_r(1,2))-(gradient_eff*xy_long_r(1,1));
                point_high_x = round((y_max-c) / gradient_eff);
                point_low_x = round((y_min-c) / gradient_eff);
                
                if frame_count == 1 && section == 1 
                    r_lane_min_x = point_low_x;
                    r_lane_min_y = y_min;
                    r_lane_max_x = point_high_x;
                    r_lane_max_y = y_max;
                end
                
                rms_dist_max = [point_high_x y_max]-[r_lane_max_x r_lane_max_y] ;
                rms_dist_min = [point_low_x y_min]-[r_lane_min_x r_lane_min_y] ;
                
                if (norm(rms_dist_max) < 5)

                    r_lane_max_x = point_high_x;
                    r_lane_max_y = y_max;

                 elseif (norm(rms_dist_max) < 40)
                     
                    r_lane_max_x = r_lane_max_x*0.75 + point_high_x*0.25 ;

                end
                
                if (norm(rms_dist_max) < 5)

                    r_lane_min_x = point_low_x;
                    r_lane_min_y = y_min;

                 elseif (norm(rms_dist_max) < 40)

                    r_lane_min_x = r_lane_min_x*0.75 + point_low_x*0.25 ;

                end

            end
        end
    end
    
            for k=-5:5
                x1=round(r_lane_min_x)+k;
                x2=round(r_lane_max_x)+k;
                y1=round(r_lane_min_y);
                y2=round(r_lane_max_y);

                xn = abs(x2-x1);
                yn = abs(y2-y1);

                % interpolate against axis with greater distance between points;
                % this guarantees statement in the under the first point!
                if (xn > yn)
                    xc = x1 : sign(x2-x1) : x2;
                    xc(xc>1280)=1280;
                    yc = round( interp1([x1 x2], [y1 y2], xc, 'linear') );
                    yc(yc>720)=720;
                else
                    yc = y1 : sign(y2-y1) : y2;
                    yc(yc>720)=720;
                    xc = round( interp1([y1 y2], [x1 x2], yc, 'linear') );
                    xc(xc>1280)=1280;
                end

                % 2-D indexes of line are saved in (xc, yc), and
                % 1-D indexes are calculated here:
                ind = sub2ind( size(vidFrame(:,:,1)), yc, xc );

                % draw line on the image (change value of '255' to one that you need)
                vidFrame(ind) = 255;
                %test(ind+838400) = 0;
                %test(ind+838400*2) = 0;

            end
     
        
    
    %% Left Lanes
   
    % Consecutive Thetas to consider in HT
    theta_range = 20;
    
    % locate desired theta index in HT data
    x =  find(T==40);
    
    % Create HT search line x points
    x = [ x : x+theta_range ]';

    % locate desired rho in hough transform data
    rho = find(R == 725);
    
    % gradient of the search line in HT
    gradient_search=1;
    
    % rho search window for a given theta in HT 
    local_window=40;
    rho_search_window=local_window/2;
    
    % Create HT search line y points
    y = [rho:gradient_search:rho+theta_range*gradient_search]';
    
    % Initial Search line points
    main_xl=x;
    main_yl=y;
    
    % Threshold to take hough value into consideration
    threshold=15;
    
    % Locally search for dense points near the initial seach line
    n = size(x,1);
    j=1;
    i=1;
    while j<=n
        mid_ind=find(H(y(i)-rho_search_window:y(i)+rho_search_window,x(i))>=threshold);%H(y(i),x(i)))

        x=[x(1:i,1); ones(size(mid_ind))*x(i,1); x(i+1:end,1)];
        y=[y(1:i,1); mid_ind+y(i,1)-rho_search_window-1; y(i+1:end,1)];
        old_i=i;
        i=size(x(1:i),1)+ size(mid_ind,1)+1;
        if H(y(old_i),x(old_i))<threshold
            x=[x(1:old_i-1);x(old_i+1:end,1)];
            y=[y(1:old_i-1);y(old_i+1:end,1)];
            i=i-1;
        end
        j=j+1;
        if i>size((x),1)
            break;
        end
    end
    
    % Hough Points for the right Lane
    P = [y x];    
    
    % Concatenate search line for both lanes 
    P_main = [main_xl main_yl];
  
    % Display inital search line(green) and collected points(blue)
    if display_info == 1 
        
        figure(hough_plot);
        hold on
        plot(T(:,P(:,2)),R(:,P(:,1)),'s','color','blue');
        plot(T(:,P_main(:,1)),R(:,P_main(:,2)),'s','color','green');


    end
    
    % check if we have any collected points and only then proceed to create
    % lines
    if size(P,1)>0
        % Create lines structure from selected hough points to image space
        lines = houghlines(edgebw,T,R,P,'FillGap',150,'MinLength',7);

        max_len = 0;
        
        if display_info == 1 
            
            figure(lines_plot);
            hold on
            
        end
        
        if exist('lines','var')==1
        
            
            for k = 1:length(lines)
               xy = [lines(k).point1; lines(k).point2];
                
               if display_info == 1 
                    
                   % Plot corresponding Lines
                   plot(xy(:,1),xy(:,2),'LineWidth',2,'Color','green');

                   % Plot beginnings and ends of lines
                   plot(xy(1,1),xy(1,2),'x','LineWidth',2,'Color','yellow');
                   plot(xy(2,1),xy(2,2),'x','LineWidth',2,'Color','red');              
               end

               % Determine the endpoints of the longest line segment
               len = norm(lines(k).point1 - lines(k).point2);
               if ( len > max_len)
                  max_len = len;
                  xy_long_l = xy;

               end
            end

     %%     

            points1_re=zeros(1,2);
            points2_re=zeros(1,2);
            lines_cell = struct2cell(lines);
            points1 = cell2mat(lines_cell(1,1,:));
            points2 = cell2mat(lines_cell(2,1,:));
            if size(points1,1) ~= 0

                for ind = 1:size(points1,3)
                    points1_re=[points1_re ; points1(1,1,ind) points1(1,2,ind)];
                    points2_re=[points2_re ; points2(1,1,ind) points2(1,2,ind)];
                end

                y_max = 681;
                y_min = 451;

                gradient = (xy_long_l(2,2)-xy_long_l(1,2))/(xy_long_l(2,1)-xy_long_l(1,1));

                if frame_count == 1
                    gradient_array_l = ones(1,10)*gradient;
                end
                gradient_array_l = [gradient_array_l(2:end) gradient];
                gradient_eff = sum(gradient_array_l)/size(gradient_array_l,2);
                gradient_old_l = gradient_eff;

                c = (xy_long_l(1,2))-(gradient_eff*xy_long_l(1,1));
                point_high_x = round((y_max-c) / gradient_eff);
                point_low_x = round((y_min-c) / gradient_eff);
                
                if frame_count == 1 && section == 1 
                    l_lane_min_x = point_low_x;
                    l_lane_min_y = y_min;
                    l_lane_max_x = point_high_x;
                    l_lane_max_y = y_max;
                end
                
                rms_dist_max = [point_high_x y_max]-[l_lane_max_x l_lane_max_y] ;
                rms_dist_min = [point_low_x y_min]-[l_lane_min_x l_lane_min_y] ;
                
                if (norm(rms_dist_max) < 5)

                    l_lane_max_x = point_high_x;
                    l_lane_max_y = y_max;

                 elseif (norm(rms_dist_max) < 40)
                     
                    l_lane_max_x = l_lane_max_x*0.75 + point_high_x*0.25 ;

                end
                
                if (norm(rms_dist_max) < 5)

                    l_lane_min_x = point_low_x;
                    l_lane_min_y = y_min;

                 elseif (norm(rms_dist_max) < 40)

                    l_lane_min_x = l_lane_min_x*0.75 + point_low_x*0.25 ;

                end

            end
        end
    end
    
            for k=-5:5
                x1=round(l_lane_min_x)+k;
                x2=round(l_lane_max_x)+k;
                y1=round(l_lane_min_y);
                y2=round(l_lane_max_y);

                xn = abs(x2-x1);
                yn = abs(y2-y1);

                % interpolate against axis with greater distance between points;
                % this guarantees statement in the under the first point!
                if (xn > yn)
                    xc = x1 : sign(x2-x1) : x2;
                    xc(xc>1280)=1280;
                    yc = round( interp1([x1 x2], [y1 y2], xc, 'linear') );
                    yc(yc>720)=720;
                else
                    yc = y1 : sign(y2-y1) : y2;
                    yc(yc>720)=720;
                    xc = round( interp1([y1 y2], [x1 x2], yc, 'linear') );
                    xc(xc>1280)=1280;
                end

                % 2-D indexes of line are saved in (xc, yc), and
                % 1-D indexes are calculated here:
                ind = sub2ind( size(vidFrame(:,:,1)), yc, xc );

                % draw line on the image (change value of '255' to one that you need)
                vidFrame(ind) = 0;
                %test(ind+838400) = 0;
                %test(ind+838400*2) = 0;

            end
        
 
%% Construct Center Line
    
            c_lane_min_x = round(l_lane_min_x + r_lane_min_x) / 2;
            c_lane_min_y = round(l_lane_min_y + r_lane_min_y) / 2;
            % Lower centerline point shoould be in the center of the image
            c_lane_max_x = (1280/2);
            c_lane_max_y = round(l_lane_max_y + r_lane_max_y) / 2;
            
                    for k=-5:5
                        x1=c_lane_min_x+k;
                        x2=c_lane_max_x+k;
                        y1=c_lane_min_y;
                        y2=c_lane_max_y;

                        xn = abs(x2-x1);
                        yn = abs(y2-y1);

                        % interpolate against axis with greater distance between points;
                        % this guarantees statement in the under the first point!
                        if (xn > yn)
                            xc = x1 : sign(x2-x1) : x2;
                            xc(xc>1280)=1280;
                            yc = round( interp1([x1 x2], [y1 y2], xc, 'linear') );
                            yc(yc>720)=720;
                        else
                            yc = y1 : sign(y2-y1) : y2;
                            yc(yc>720)=720;
                            xc = round( interp1([y1 y2], [x1 x2], yc, 'linear') );
                            xc(xc>1280)=1280;
                        end

                        % 2-D indexes of line are saved in (xc, yc), and
                        % 1-D indexes are calculated here:
                        ind = sub2ind( size(vidFrame(:,:,1)), yc, xc );

                        % draw line on the image (change value of '255' to one that you need)
                        vidFrame(ind) = 155;

                        %imshow(im2bw(max(test(:,:,1),test(:,:,2)),0.65))
                    end

        


%%
% End of Lane construction


    if display_info==true
     figure
         imshow(vidFrame);
    end
    
    if (just_view == true) && (display_info == false)
        imshow(vidFrame);
        frame_count=frame_count+1
        hold on
        pause(1/30);
        
    end
    if iter_once == true
        break;
    end
    
    if just_view == false
        Lane(:,:,:,frame_count)=im2double(vidFrame);


        if frame_count==100
            v = VideoWriter(sprintf('%s_%d.avi','/home/vendetta/Documents/ENPM673_HW1/Lane',section));
            v.FrameRate=25;
            open(v);
            writeVideo(v,double(Lane));
            close(v);
            frame_count=1;
            section=section+1
            Lane = zeros(720,1280,3);
        end
    end
    
    frame_count=frame_count+1
    
    
    
end

if iter_once ~= true
v = VideoWriter(sprintf('%s_%d.avi','/home/vendetta/Documents/ENPM673_HW1/Lane',section));
        v.FrameRate=25;
        open(v);
        writeVideo(v,double(Lane));
        close(v);
        frame_count=1;
        section=section+1
%%
clear test
clear vidObj
clear Lane
clear vidFrame
clear Fused
clear v

all_frames=zeros(720,1280,3,1);
for i=1:section-1
    mov=VideoReader(sprintf('%s_%d.avi','/home/vendetta/Documents/ENPM673_HW1/Lane',i));
    curr_clip=read(mov);
    all_frames=cat(4,all_frames,curr_clip);
end
%%
v = VideoWriter('/home/vendetta/Documents/ENPM673_HW1/Lane_complete.mp4');
        v.FrameRate=25;
        open(v);
        writeVideo(v,all_frames);
        close(v);

end


