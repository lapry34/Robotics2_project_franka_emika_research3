function plot3dMotion(p_list, p_start, p_end, p_sing, dt, useSlider)
    % plot3dMotion  Animate or interactively browse a 3D end-effector trajectory.
    %   plot3dMotion(p_list,p_start,p_end,p_sing,dt)  
    %     → automated animation at timestep dt.
    %   plot3dMotion(..., true)  
    %     → adds a slider for manual scrubbing.

    if nargin<6, useSlider = false; end

    % prepare figure
    fig = figure('Name','3D End-Effector Motion','NumberTitle','off');
    ax  = axes(fig);
    hold(ax,'on'), grid(ax,'on'), view(3), axis equal
    xlabel('X (m)'), ylabel('Y (m)'), zlabel('Z (m)')
    title('End-Effector 3D Trajectory')

    % static markers
    hTrail = plot3(ax, NaN,NaN,NaN, 'LineWidth',1.5, 'Color',[.5 .5 .5]);
    hStart = scatter3(ax, p_start(1),p_start(2),p_start(3), 50,'g','filled');
    hEnd   = scatter3(ax, p_end(1),  p_end(2),  p_end(3),   50,'b','filled');
    hSing  = scatter3(ax, p_sing(1), p_sing(2), p_sing(3),  80,'r','filled');
    hPoint = scatter3(ax, p_list(1,1), p_list(2,1), p_list(3,1), 60,'k','filled');

    % pad axes around the start point
    dx = 0.5;
    xlim([p_start(1)-dx, p_start(1)+dx]);
    ylim([p_start(2)-dx, p_start(2)+dx]);
    zlim([p_start(3)-dx, p_start(3)+dx]);

    [nDims, nSteps] = size(p_list);

    if ~useSlider
        % ---- automated playback ----
        for k = 1:nSteps
            set(hTrail, 'XData', p_list(1,1:k), ... 
                        'YData', p_list(2,1:k), ...
                        'ZData', p_list(3,1:k));
            set(hPoint, 'XData', p_list(1,k), ...
                        'YData', p_list(2,k), ...
                        'ZData', p_list(3,k));
            drawnow;
            pause(dt);
        end
    else
        % ---- interactive slider ----
        uicontrol(fig,'Style','slider', ...
            'Units','normalized','Position',[0.15 0.02 0.7 0.04], ...
            'Min',1,'Max',nSteps,'Value',1, ...
            'SliderStep',[1/(nSteps-1) , 10/(nSteps-1)], ...
            'Callback',@(src,~) sliderCallback(src, hTrail, hPoint, p_list));
        
        uicontrol(fig,'Style','text', ...
            'Units','normalized','Position',[0.87 0.02 0.1 0.04], ...
            'String','Frame');
    end

end


% -------------------------------------------------------------------------
function sliderCallback(src, hTrail, hPoint, p_list)
% Called whenever the slider moves
    k = round(get(src,'Value'));
    set(hTrail, 'XData', p_list(1,1:k), ...
                'YData', p_list(2,1:k), ...
                'ZData', p_list(3,1:k));
    set(hPoint, 'XData', p_list(1,k), ...
                'YData', p_list(2,k), ...
                'ZData', p_list(3,k));
    drawnow;
end
