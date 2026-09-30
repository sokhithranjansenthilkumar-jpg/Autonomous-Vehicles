classdef TechnicalView < handle
    %TECHNICALVIEW Live diagnostics; controls affect display, never integration.
    properties (Access=private)
        Figure
        Axes
        CostAxes
        Telemetry
        Inspector
        CandidateMenu
        PauseButton
        Video
        Config
        FrameStride=2
        FrameCount=0
        FrameClock
        LastDisplayedTimeS=0
        SnapshotSaved=false
        LastFrame
        Finished=false
    end
    properties
        Paused=false
        ResetRequested=false
        Cancelled=false
        ShowCandidates=true
        ShowUncertainty=true
        PlaybackSpeed=1
    end
    methods
        function obj=TechnicalView(cfg)
            obj.Config=cfg; obj.PlaybackSpeed=cfg.visualization.playbackSpeed;
            obj.Paused=cfg.visualization.startPaused;
            obj.ShowCandidates=cfg.visualization.showCandidates;
            obj.ShowUncertainty=cfg.visualization.showUncertainty;
            bg=[.025 .035 .045]; silver=[.78 .84 .89];
            obj.Figure=figure('Name','PathWeaver | BU SIH-26 Phase 2','Color',bg, ...
                'Position',[40 60 1280 720],'MenuBar','none','ToolBar','none', ...
                'KeyPressFcn',@(~,event)obj.key(event.Key), ...
                'CloseRequestFcn',@(~,~)obj.cancel());
            if cfg.visualization.video, obj.Figure.Resize='off'; end
            annotation(obj.Figure,'textbox',[.045 .945 .92 .045],'String', ...
                sprintf('PATHWEAVER   /   Bennett University   /   SIH26037      %s | %s | seed %d', ...
                cfg.scenario.preset,cfg.mode,cfg.seed),'Color',[0 .88 .86], ...
                'EdgeColor','none','FontSize',15,'FontWeight','bold','Interpreter','none');
            obj.Axes=axes(obj.Figure,'Position',[.055 .445 .91 .445], ...
                'Color',bg,'XColor',silver,'YColor',silver,'FontSize',10);
            obj.Telemetry=annotation(obj.Figure,'textbox',[.035 .045 .33 .325], ...
                'String','Initialising simulated world-state input','Color',silver, ...
                'BackgroundColor',[.05 .07 .09],'EdgeColor',[.2 .35 .4], ...
                'FontName','Menlo','FontSize',10,'Interpreter','none');
            obj.CostAxes=axes(obj.Figure,'Position',[.485 .065 .21 .275], ...
                'Color',bg,'XColor',silver,'YColor',silver,'FontSize',9);
            obj.CandidateMenu=uicontrol(obj.Figure,'Style','popupmenu','Units','normalized', ...
                'Position',[.74 .305 .235 .04],'String',{'Selected trajectory'}, ...
                'Callback',@(~,~)obj.refreshInspector());
            obj.Inspector=annotation(obj.Figure,'textbox',[.735 .055 .245 .25], ...
                'String','','Color',silver,'EdgeColor',[.2 .35 .4], ...
                'FontName','Menlo','FontSize',10,'Interpreter','none');
            obj.PauseButton=uicontrol(obj.Figure,'Style','pushbutton','Units','normalized', ...
                'Position',[.05 .385 .1 .035],'String','Pause / Space','Callback',@(~,~)obj.key('space'));
            uicontrol(obj.Figure,'Style','pushbutton','Units','normalized', ...
                'Position',[.16 .385 .09 .035],'String','Reset / R','Callback',@(~,~)obj.key('r'));
            uicontrol(obj.Figure,'Style','checkbox','Units','normalized', ...
                'Position',[.27 .385 .16 .035],'String','Uncertainty / U','Value',obj.ShowUncertainty, ...
                'Callback',@(src,~)obj.toggle('uncertainty',src.Value));
            uicontrol(obj.Figure,'Style','checkbox','Units','normalized', ...
                'Position',[.44 .385 .16 .035],'String','Candidates / C','Value',obj.ShowCandidates, ...
                'Callback',@(src,~)obj.toggle('candidates',src.Value));
            uicontrol(obj.Figure,'Style','popupmenu','Units','normalized', ...
                'Position',[.62 .385 .12 .035],'String',{'0.5x','1x','2x','4x'}, ...
                'Value',find([.5 1 2 4]==obj.PlaybackSpeed,1), ...
                'Callback',@(src,~)obj.setSpeed(src.Value));
            annotation(obj.Figure,'textbox',[.76 .372 .22 .045],'String','Esc: stop | R: clean reset', ...
                'Color',silver,'EdgeColor','none','FontSize',10);
            annotation(obj.Figure,'textbox',[.035 .003 .94 .04],'String', ...
                ['PathWeaver v0.1 consumes simulated world-state data. Multi-sensor perception, detection and sensor fusion ' ...
                'are planned for later versions and are not claimed by this prototype.'], ...
                'Color',[.6 .67 .72],'EdgeColor','none','FontSize',8,'Interpreter','none');
            if cfg.visualization.video
                obj.Video=VideoWriter(fullfile(cfg.outputDirectory,'pathweaver_demo.mp4'),'MPEG-4');
                obj.Video.FrameRate=1/(cfg.dt*obj.FrameStride); open(obj.Video);
            end
            obj.FrameClock=tic;
        end
        function update(obj,frame)
            obj.LastFrame=frame; obj.FrameCount=obj.FrameCount+1;
            drawnow;
            while obj.Paused && ~obj.ResetRequested && ~obj.Cancelled
                obj.PauseButton.String='Resume / Space'; pause(.05); drawnow;
            end
            if obj.ResetRequested, error('PathWeaver:ResetRequested','Reproducible reset requested.'); end
            if obj.Cancelled || ~isgraphics(obj.Figure), error('PathWeaver:DemoStopped','Demo stopped by user.'); end
            obj.PauseButton.String='Pause / Space';
            terminal=frame.collision || frame.completed || frame.ego.timestampS>=obj.Config.maxSimulationTime-1e-8;
            if mod(obj.FrameCount-1,obj.FrameStride)~=0 && ~terminal, return; end
            ax=obj.Axes; cla(ax); hold(ax,'on'); cfg=obj.Config;
            s=frame.scenario; planner=frame.planner;
            fill(ax,[s.leftBoundaryWorldM(:,1);flipud(s.rightBoundaryWorldM(:,1))], ...
                [s.leftBoundaryWorldM(:,2);flipud(s.rightBoundaryWorldM(:,2))], ...
                [.10 .13 .16],'EdgeColor','none');
            plot(ax,s.leftBoundaryWorldM(:,1),s.leftBoundaryWorldM(:,2),'Color',[.55 .63 .7],'LineWidth',1.4);
            plot(ax,s.rightBoundaryWorldM(:,1),s.rightBoundaryWorldM(:,2),'Color',[.55 .63 .7],'LineWidth',1.4);
            candidates=planner.candidateTrajectories;
            if obj.ShowCandidates
                % Evenly spaced subset; every candidate remains inspectable below.
                indices=unique(round(linspace(1,numel(candidates),min(14,numel(candidates)))));
                for k=indices
                    c=candidates(k); colour=[.42 .46 .50];
                    if ~c.isFeasible, colour=[.57 .21 .24]; end
                    plot(ax,c.positionsWorldM(:,1),c.positionsWorldM(:,2),'Color',colour,'LineWidth',.65);
                end
            end
            selected=planner.selectedTrajectory; colour=[0 .88 .86];
            if ~selected.isFeasible, colour=[.95 .28 .30]; end
            plot(ax,selected.positionsWorldM(:,1),selected.positionsWorldM(:,2), ...
                'Color',colour,'LineWidth',2.8);
            for a=1:numel(frame.predictions)
                p=frame.predictions(a);
                plot(ax,p.expectedPositionsWorldM(:,1),p.expectedPositionsWorldM(:,2),'--','Color',[.9 .65 .25]);
                if obj.ShowUncertainty
                    for horizon=[1 2 3]
                        [~,index]=min(abs(p.futureTimestampsS-planner.timestampS-horizon));
                        drawEllipse(ax,p.expectedPositionsWorldM(index,:),p.positionCovariancesWorldM2(:,:,index));
                        text(ax,p.expectedPositionsWorldM(index,1)+.2,p.expectedPositionsWorldM(index,2)+.3, ...
                            sprintf('+%.1fs',p.futureTimestampsS(index)-planner.timestampS), ...
                            'Color',[.95 .74 .4],'FontSize',8,'Clipping','on');
                    end
                end
            end
            for a=1:numel(s.agents)
                agent=s.agents(a);
                drawDisc(ax,agent.positionWorldM,agent.collisionRadiusM,[.92 .63 .20],true);
                text(ax,agent.positionWorldM(1),agent.positionWorldM(2)+.9, ...
                    sprintf('%s #%d',agent.class,agent.id),'Color',[.88 .90 .92], ...
                    'FontSize',9,'Interpreter','none','Clipping','on');
            end
            obs=s.staticObstacles; drawDisc(ax,obs.positionWorldM,obs.geometry.radiusM,[.65 .25 .18],true);
            text(ax,obs.positionWorldM(1),obs.positionWorldM(2)-1.25,'POTHOLE','Color',[.85 .5 .4],'FontSize',9,'Clipping','on');
            [centres,radius]=pathweaver.core.egoFootprint(frame.ego.positionWorldM,frame.ego.headingRad,cfg);
            for j=1:3, drawDisc(ax,centres(:,:,j),radius,[0 .55 .56],false); end
            corners=[-1 -1;1 -1;1 1;-1 1].*[cfg.ego.lengthM/2 cfg.ego.widthM/2];
            h=frame.ego.headingRad; rotation=[cos(h) -sin(h);sin(h) cos(h)];
            body=corners*rotation'+frame.ego.positionWorldM;
            fill(ax,body(:,1),body(:,2),[0 .7 .72],'EdgeColor',[0 1 .95]);
            plot(ax,s.goalPositionWorldM(1),s.goalPositionWorldM(2),'p','MarkerSize',14, ...
                'MarkerFaceColor',[.25 .85 .5],'MarkerEdgeColor','none');
            left=max(0,min(s.roadXWorldM(end)-48,frame.ego.positionWorldM(1)-12));
            axis(ax,'equal'); xlim(ax,[left left+48]); ylim(ax,[-6 6]); grid(ax,'on');
            ax.GridColor=[.3 .4 .45]; ax.GridAlpha=.3;
            title(ax,'Simulated world-state input  |  cyan: selected  /  gray: feasible  /  red: rejected  /  amber: prediction', ...
                'Color',[.8 .86 .9],'FontSize',10);
            xlabel(ax,'x world (m)'); ylabel(ax,'y world (m)');
            status="RUNNING"; if frame.completed, status="GOAL REACHED"; elseif frame.collision, status="COLLISION";
            elseif terminal, status="TIMEOUT"; end
            obj.Telemetry.String=sprintf(['%s | %.2f s | %.1f km/h\nState: %s\nReason: %s\n' ...
                'Clearance: %.3f m (swept footprint)\nCV TTC: %s\nRisk score: %.3f (uncalibrated)\n' ...
                'Planner: %.2f ms | plan age %.2f s\nFeasible: %d / %d | selected J: %.2f\n' ...
                'Control: a %.2f m/s^2 | emergency override %d\n' ...
                'Ellipses: 2-sigma at +1 / +2 / +3 s'], ...
                status,frame.ego.timestampS,frame.ego.speedMps*3.6,frame.behavior.name,frame.behavior.reason, ...
                frame.clearanceM,pathweaver.planning.ttcLabel(planner.minimumTtcS,cfg.horizon),planner.riskScore, ...
                planner.planningLatencyS*1000,frame.ego.timestampS-planner.timestampS, ...
                sum([candidates.isFeasible]),numel(candidates),selected.totalCost, ...
                frame.ego.accelerationMps2,frame.emergencyOverride);
            cla(obj.CostAxes);
            if selected.isFeasible
                names=fieldnames(selected.weightedCostTerms); values=struct2array(selected.weightedCostTerms);
                barh(obj.CostAxes,values,'FaceColor',[0 .64 .65]);
                yticks(obj.CostAxes,1:numel(names)); yticklabels(obj.CostAxes,names);
                obj.CostAxes.TickLabelInterpreter='none'; obj.CostAxes.YDir='reverse';
                title(obj.CostAxes,'Selected weighted costs','Color',[.8 .86 .9],'FontSize',10);
            else
                text(obj.CostAxes,.05,.5,'INFEASIBLE: bounded brake; safety not guaranteed', ...
                    'Units','normalized','Color',[.95 .4 .4],'FontSize',10);
                obj.CostAxes.XTick=[]; obj.CostAxes.YTick=[];
            end
            labels=cell(1,numel(candidates)+1); labels{1}='Selected trajectory';
            for k=1:numel(candidates)
                labels{k+1}=sprintf('%02d | %s',k,candidateStatus(candidates(k)));
            end
            obj.CandidateMenu.String=labels; obj.CandidateMenu.Value=min(obj.CandidateMenu.Value,numel(labels));
            obj.refreshInspector();
            delay=(frame.ego.timestampS-obj.LastDisplayedTimeS)/obj.PlaybackSpeed-toc(obj.FrameClock);
            if delay>0, pause(delay); end
            drawnow;
            if ~isempty(obj.Video), writeVideo(obj.Video,getframe(obj.Figure)); end
            if ~obj.SnapshotSaved && frame.ego.timestampS>=4.5
                captured=getframe(obj.Figure);
                imwrite(captured.cdata,fullfile(cfg.outputDirectory,'pathweaver_snapshot.png'));
                obj.SnapshotSaved=true;
            end
            obj.LastDisplayedTimeS=frame.ego.timestampS; obj.FrameClock=tic;
            if terminal, obj.Finished=true; obj.PauseButton.String='Finished'; end
        end
        function refreshInspector(obj)
            if isempty(obj.LastFrame), return; end
            i=obj.CandidateMenu.Value-1; planner=obj.LastFrame.planner;
            if i==0, candidate=planner.selectedTrajectory; label='Selected';
            else, candidate=planner.candidateTrajectories(min(i,numel(planner.candidateTrajectories))); label=sprintf('Candidate %02d',i); end
            obj.Inspector.String=sprintf(['%s\n%s\nJ: %.3f | end speed: %.2f m/s\n' ...
                'End: (%.1f, %.1f) m\nMax curvature: %.3f /m\nEmergency profile: %d\n\n' ...
                'Ground truth, not sensors.\nNo road-safety guarantee.'],label,candidateStatus(candidate), ...
                candidate.totalCost,candidate.speedsMps(end),candidate.positionsWorldM(end,:), ...
                max(abs(candidate.curvaturesPerM)),candidate.isEmergencyBraking);
        end
        function key(obj,key)
            switch key
                case 'space', obj.Paused=~obj.Paused;
                case 'r'
                    if obj.Finished
                        cfg=obj.Config; obj.dispose();
                        runPathWeaverDemo(preset=cfg.scenario.preset,seed=cfg.seed,plannerMode=cfg.mode, ...
                            visualization=true,video=cfg.visualization.video,playbackSpeed=obj.PlaybackSpeed, ...
                            maximumSimulationTime=cfg.maxSimulationTime,outputDirectory=string(cfg.outputDirectory));
                    else
                        obj.ResetRequested=true;
                    end
                case 'escape', obj.Cancelled=true;
                case 'u', obj.ShowUncertainty=~obj.ShowUncertainty;
                case 'c', obj.ShowCandidates=~obj.ShowCandidates;
            end
        end
        function toggle(obj,name,value)
            if strcmp(name,'uncertainty'), obj.ShowUncertainty=logical(value);
            else, obj.ShowCandidates=logical(value); end
        end
        function setSpeed(obj,index)
            speeds=[.5 1 2 4]; obj.PlaybackSpeed=speeds(index);
        end
        function cancel(obj)
            obj.Cancelled=true; if isgraphics(obj.Figure), delete(obj.Figure); end
        end
        function close(obj)
            if ~isempty(obj.Video), close(obj.Video); obj.Video=[]; end
        end
        function dispose(obj)
            obj.close(); if isgraphics(obj.Figure), delete(obj.Figure); end
        end
    end
end
function drawDisc(ax,centre,radius,colour,filled)
angle=linspace(0,2*pi,40); x=centre(1)+radius*cos(angle); y=centre(2)+radius*sin(angle);
if filled, fill(ax,x,y,colour,'EdgeColor','none'); else, plot(ax,x,y,'Color',colour,'LineWidth',.6); end
end
function drawEllipse(ax,centre,covariance)
[v,d]=eig((covariance+covariance')/2); angle=linspace(0,2*pi,48);
points=centre'+2*v*sqrt(max(d,0))*[cos(angle);sin(angle)];
plot(ax,points(1,:),points(2,:),'Color',[.85 .62 .26],'LineWidth',.65);
end
function value=candidateStatus(candidate)
if candidate.isFeasible, value="feasible"; else, value=candidate.rejectionReason; end
end
