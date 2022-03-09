classdef PoseViewerWithSwitches < HelperPoseViewer
    properties
        imuFs = 160;    % IMU (accelerometer and gyroscope)
        magFs = 40;     % magnetometer
        gpsFs = 5;      % GPS
        barFs = 1;      % barometer
        altFs = 50;     % altimeter
        uwbFs = 40;     % Ultra Wide Band
        visFs = 25;     % Vision system (ArUco marker)
    end
    properties (Hidden)
        thefig
    end
    methods (Access = protected)
        function setupImpl(obj, varargin)
            obj.thefig.Visible = true;
            setupImpl@HelperPoseViewer(obj, varargin{:});
        end
        
        function createAppWindow(obj)
            fig = figure('Name', 'Pose Viewer', ...
                'NumberTitle', 'off', ...
                'DockControls','off', ...
                'Units', 'normalized', ...
                'OuterPosition', [0 0.25 0.5 0.5], ...
                'Visible', 'off', ...
                'HandleVisibility', 'on', ...
                'NextPlot', 'new', ...
                'IntegerHandle', 'off', ...
                'CloseRequestFcn', @(x,~)set(x,'Visible', 'off'));
            u1 = uipanel('Title', 'Pose', 'Parent', fig, 'Position', ...
                [0 0 0.75 1]);
            u2 = uipanel('Title', "Sensors", "Parent", fig, 'Position', ...
                [0.75 0 0.25 1]);
            obj.AppWindow = u1;
            obj.thefig = fig;
            
            ac = uicontrol(u2, 'Style', 'Checkbox', 'String', 'Decimation', ...
                'Units', 'normalized', 'Callback', @checkboxcb);
            ac.Position(2:3) = [0.8 0.5];
            ac.Value = true;
            af = uicontrol(u2, 'Style', 'popupmenu', 'String', ...
                ["1 Sample", "2 Samples", "4 Samples", "8 Samples", "16 Samples"], ...
                'Units', 'normalized', 'tag', 'DecimationFactor', ...
                'Callback', @pulldowncb);
            af.Position(1:2) = [0.6 0.8];
            
            gc = uicontrol(u2, 'Style', 'Checkbox', 'String', 'Accelerometer', ...
                'Units', 'normalized', 'Callback', @checkboxcb);
            gc.Position(2:3) = [0.7 0.5];
            gc.Value = true;
            gf = uicontrol(u2, 'Style', 'popupmenu', 'String', ...
                [num2str(obj.imuFs)+" Hz"], ...
                'Units', 'normalized', 'tag', 'AccelerometerSampleRate', ...
                'Callback', @pulldowncb);
            gf.Position(1:2) = [0.6 0.7];
            
            mc = uicontrol(u2, 'Style', 'Checkbox', 'String', 'Magnetometer', ...
                'Units', 'normalized', 'Callback', @checkboxcb);
            mc.Position(2:3) = [0.6 0.5];
            mc.Value = true;
            mf = uicontrol(u2, 'Style', 'popupmenu', 'String', ...
                [num2str(obj.magFs)+" Hz", num2str(floor(obj.magFs/2))+" Hz"], ...
                'Units', 'normalized', 'tag', 'MagnetometerSampleRate', ...
                'Callback', @pulldowncb);
            mf.Position(1:2) = [0.6 0.6];
            
            gpsc = uicontrol(u2, 'Style', 'Checkbox', 'String', 'GPS', ...
                'Units', 'normalized', 'Callback', @checkboxcb);
            gpsc.Position(2:3) = [0.5 0.5];
            gpsc.Value = true;
            gpsf = uicontrol(u2, 'Style', 'popupmenu', 'String', ...
                [num2str(obj.gpsFs)+" Hz", num2str(floor(obj.gpsFs/5))+" Hz"], ...
                'Units', 'normalized', 'tag', 'GPSSampleRate', ...
                'Callback', @pulldowncb);
            gpsf.Position(1:2) = [0.6 0.5];
            
            uwbc = uicontrol(u2, 'Style', 'Checkbox', 'String', 'UltraWideBand', ...
                'Units', 'normalized', 'Callback', @checkboxcb);
            uwbc.Position(2:3) = [0.4 0.5];
            uwbc.Value = true;
            uwbf = uicontrol(u2, 'Style', 'popupmenu', 'String', ...
                [num2str(obj.uwbFs)+" Hz", num2str(floor(obj.uwbFs/2))+" Hz"], ...
                'Units', 'normalized', 'tag', 'uwbSampleRate', ...
                'Callback', @pulldowncb);
            uwbf.Position(1:2) = [0.6 0.4];
            
            visc = uicontrol(u2, 'Style', 'Checkbox', 'String', 'Vision', ...
                'Units', 'normalized', 'Callback', @checkboxcb);
            visc.Position(2:3) = [0.3 0.5];
            visc.Value = true;
            visf = uicontrol(u2, 'Style', 'popupmenu', 'String', ...
                [num2str(obj.visFs)+" Hz", num2str(floor(obj.uwbFs/5*2))+" Hz"], ...
                'Units', 'normalized', 'tag', 'visSampleRate', ...
                'Callback', @pulldowncb);
            visf.Position(1:2) = [0.6 0.3];
            
            barc = uicontrol(u2, 'Style', 'Checkbox', 'String', 'Barometer', ...
                'Units', 'normalized', 'Callback', @checkboxcb);
            barc.Position(2:3) = [0.2 0.5];
            barc.Value = true;
            barf = uicontrol(u2, 'Style', 'popupmenu', 'String', ...
                [num2str(obj.barFs)+" Hz", num2str(obj.barFs/2)+" Hz"], ...
                'Units', 'normalized', 'tag', 'barSampleRate', ...
                'Callback', @pulldowncb);
            barf.Position(1:2) = [0.6 0.2];
            
            altc = uicontrol(u2, 'Style', 'Checkbox', 'String', 'Altimeter', ...
                'Units', 'normalized', 'Callback', @checkboxcb);
            altc.Position(2:3) = [0.1 0.5];
            altc.Value = true;
            altf = uicontrol(u2, 'Style', 'popupmenu', 'String', ...
                [num2str(obj.altFs)+" Hz", num2str(floor(obj.altFs/2))+" Hz"], ...
                'Units', 'normalized', 'tag', 'altSampleRate', ...
                'Callback', @pulldowncb);
            altf.Position(1:2) = [0.6 0.1];
            
            % Defaults
            fig.UserData.Decimation = true;
            fig.UserData.Accelerometer = true;
            fig.UserData.Magnetometer = true;
            fig.UserData.GPS = true;
            fig.UserData.UltraWideBand = true;
            fig.UserData.Vision = true;
            fig.UserData.Barometer = true;
            fig.UserData.Altimeter = true;
            
            fig.UserData.DecimationFactor = 1;
            fig.UserData.AccelerometerSampleRate = obj.imuFs;
            fig.UserData.MagnetometerSampleRate = obj.magFs;
            fig.UserData.GPSSampleRate = obj.gpsFs;
            fig.UserData.uwbSampleRate = obj.uwbFs;
            fig.UserData.visSampleRate = obj.visFs;
            fig.UserData.barSampleRate = obj.barFs;
            fig.UserData.altSampleRate = obj.altFs;
        end
    end
    
end

function checkboxcb(src, ~)
[~,f] = gcbo;
f.UserData.(src.String) = src.Value;
end

function pulldowncb(src, ~)
[~,f] = gcbo;
str = strsplit(src.String{src.Value});
f.UserData.(src.Tag) = str2double(str{1});
end
