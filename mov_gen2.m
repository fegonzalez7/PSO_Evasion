function mov = mov_gen2(varargin)
s = ' ';
switch nargin
    case 2
        movType = varargin{1};
        robt = varargin{2};
        veldata = 'v400';
        zonedata = 'z0';
        tooldata = 'tool0';
        wobj = 'wobj0';
    case 4
        movType = varargin{1};
        robt = varargin{2};
        vel = varargin{3};
        zone = varargin{4};
        veldata = ['v' num2str(vel)];
        zonedata = ['z' num2str(zone)];
        tooldata = 'tool0';
        wobj = 'wobj0';
    case 5
        movType = varargin{1};
        robt = varargin{2};
        vel = varargin{3};
        zone = varargin{4};
        tool = varargin{5};
        veldata = ['v' num2str(vel)];
        zonedata = ['z' num2str(zone)];
        tooldata = tool;
        wobj = 'wobj0';       
    otherwise
        disp('wrong inputs')
end

if strcmp(movType,'J')
    movT = 'MoveJ';
elseif strcmp(movType,'L')
    movT = 'MoveL';
end
mov = [movT s robt ',' veldata ',' zonedata ',' tooldata '\\WObj:=' wobj ';'];