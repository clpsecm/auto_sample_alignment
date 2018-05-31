clear; close all; clc;

%% ===== Global parameters ====== %%
L = 2; %-length of probe
WorldXLim = 1.5*[-2*L,L]; %-used for plot xlim 
WorldYLim = 1.5*[-2*L,L]; %-used for plot ylim

num_points = 1;
num_rot = 3;

PLOT_ON = true; %-{'true','false'}


%% ======= Input angles and intial rotation center ====== %%
%-Create random point in unit disk
pts_rt = rand(2,num_points);
pts_rt(1,:) = sqrt(pts_rt(1,:));
pts_rt(2,:) = 2*pi*pts_rt(2,:);
pts_xy = [pts_rt(1,:).*cos(pts_rt(2,:)); ...
          pts_rt(1,:).*sin(pts_rt(2,:)) ];


%-Generate rotation
sub1 = ceil(sqrt(num_rot));
sub2 = ceil(num_rot/sub1);
% angle_rad = (2*pi)*(rand(1,num_rot));
angle_rad = [40,40,40]*(pi/180); 

rotcenter_xy = 2*rand(2,1)-1; 
%-[Note] In practice
%  rotcenter_xy =  [relative dist. from probe left end] + [-L/2;L/2]


%-Save original parameters
rotcenter_origin = rotcenter_xy;
pts_origin = pts_xy;


%% ====== Effective area ===== %% 
%-Create Ceff area

frame_ref = linspace(0,2*pi,1000);
Ceff_area = L/2*[cos(frame_ref);sin(frame_ref)];

%-Create probe location
probe_area = [linspace(-L/2,L/2,100);...
              linspace(L/2,L/2,100)];

%-Save original parameters
Ceff_origin = Ceff_area;


%% ======= Setup rotation operation ====== %%
%-[Note] Rotate the stage clockwise by assigned angle
rotate = @(in_xy,center_xy,angle_rad) ...
          [ cos(angle_rad) sin(angle_rad);...
           -sin(angle_rad) cos(angle_rad) ]*...
           (in_xy - repmat(center_xy,1,size(in_xy,2))) + ...
           repmat(center_xy,1,size(in_xy,2));





%% ===== Plot simluated rotaion + traslation ===== %%
if PLOT_ON
    figure(1);
    subplot(sub1,sub2,1)

    hold on;

    xlim(WorldXLim);
    ylim(WorldYLim);


    plot(probe_area(1,:),probe_area(2,:),'.k','MarkerSize',10);
    plot(Ceff_area(1,:),Ceff_area(2,:),'.','MarkerSize',3);
    scatter(pts_xy(1,:),pts_xy(2,:),40,'filled')
    scatter(rotcenter_xy(1), rotcenter_xy(2),'x','LineWidth',2.5);

    title('Original')
    axis('square')
    legend({'Probe','Ceff','Initial Point','Initial Center'},...
                    'Location','Southwest');
end

for I = 1:num_rot
    if PLOT_ON
        subplot(sub1,sub2,I+1)
        hold on;
        xlim(WorldXLim)
        ylim(WorldYLim)
    
        %-Pre-rotate layout
        plot(probe_area(1,:),probe_area(2,:),'.k','MarkerSize',10);
        plot(Ceff_area(1,:),Ceff_area(2,:),'.','MarkerSize',3)
        scatter(pts_xy(1,:),pts_xy(2,:),40,'filled')
        scatter(rotcenter_xy(1), rotcenter_xy(2),'x','LineWidth',2.5);
    end
    
    %-Rotate by rotational stage center
    Ceff_area = rotate(Ceff_area,rotcenter_xy,angle_rad(I));
    pts_xy = rotate(pts_xy,rotcenter_xy,angle_rad(I));
    
    if PLOT_ON
        plot(Ceff_area(1,:),Ceff_area(2,:),'.','MarkerSize',3)
        scatter(pts_xy(1,:),pts_xy(2,:),40,'filled')
    end

    %-Shift by assigned value
    %-[Note]: Rotational center also shifted
    dxy = [cos(angle_rad(I))-1, sin(angle_rad(I));...
          -sin(angle_rad(I)),  cos(angle_rad(I))-1]*rotcenter_xy;
    Ceff_area = bsxfun(@plus, Ceff_area, dxy);
    pts_xy    = bsxfun(@plus, pts_xy,    dxy);
    rotcenter_xy = bsxfun(@plus, rotcenter_xy, dxy);

    %-Post-shift layout
    if PLOT_ON
        plot(Ceff_area(1,:),Ceff_area(2,:),'.','MarkerSize',3)
        scatter(pts_xy(1,:),pts_xy(2,:),40,'filled')
        scatter(rotcenter_xy(1), rotcenter_xy(2),'x','LineWidth',2.5);
    
        title(strcat('Relative Rotation #',num2str(I),{' by '},num2str(angle_rad(I)*180/pi,3),'^o'))
        axis('square')
        legend({'Probe','Pre-rotation Ceff','Pre-rotation Point','Pre-rotation Center',...
                        'Rotated Ceff','Rotated Point',...
                        'Post-shift Ceff','Post-shift Point','Post-shift Center'},...
                        'Location','Southwest');
    end
end




