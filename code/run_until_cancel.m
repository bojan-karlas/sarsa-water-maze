function run_until_cancel( arena )
% RUN_UNTIL_CANCEL Run a single experiment and display the progression of
% the agent through the arena. The path is shown in an interactive graph
% window which, when closed, causes the experiment to halt.

    cancelled = false;

    figure(1);
    clf;
    plot_arena(arena);
    drawnow;
    uicontrol('Style', 'pushbutton', 'String', 'Stop', 'Callback', @Button_Callback);
    
    while not(arena.step()) && not(cancelled)
        plot_last_step(arena);
        plot_wall_crossings(arena);
        drawnow;
    end
    
    if not(cancelled), close(1); end;
    
    function Button_Callback(hObject, eventdata, handles)
        cancelled = true;
        close(1);
        drawnow;
    end

end
