function axes = snake_fk_plot(g, virtualChassis, axes, is_init)
%snake_fk_plot given a list of transformations, plot them in axes handle

    N = size(g,3);
    visLineDataX = zeros(2,N-1);
    visLineDataY = zeros(2,N-1);
    visLineDataZ = zeros(2,N-1);
    
    center = virtualChassis(1:3,4);
    virtualChassis_R = virtualChassis(1:3,1:3);

    for i=1:N-1
        [~, p1] = TransToRp(g(:,:,i)); p1 = virtualChassis_R'*(p1 - center);
        [~, p2] = TransToRp(g(:,:,i+1)); p2 = virtualChassis_R'*(p2 - center);
        visLineDataX(:,i) = [p1(1);p2(1)];
        visLineDataY(:,i) = [p1(2);p2(2)];
        visLineDataZ(:,i) = [p1(3);p2(3)];
    end
    if (is_init == 1)
        
        plot3(axes, visLineDataX, visLineDataY,visLineDataZ ,'b','LineWidth',8);
    else
        for i=1:N-1
            set(axes.Children(i),'XData',visLineDataX(:,i)','YData',visLineDataY(:,i)','ZData',visLineDataZ(:,i)');
        end
        % set these later
        xbound = [-0.2;1.2];
        ybound = [-0.5; 0.5];
        zbound = [-0.5; 0.5];

        axis([xbound(1) xbound(2) ybound(1) ybound(2) zbound(1) zbound(2)]);
        grid on;
        gca.DataAspectRatio = [1 1 1];
%         az = 45;
%         el = 30;
%         view(az, el);

        
    end
end

