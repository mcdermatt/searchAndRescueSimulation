%Matt McDermott
%ME149: Colaborative Robotics Final Simulation

timestart = cputime;

%To-Do----
%make vicims outside obstacles
%fire wavefront
%danger barrier
%timesteps
%drone agent
%human agent
%optimal path planning - make heatmap
%add in max number of vicims to carry
%scoring metric
%monte carlo- get best strategy
%detection probabilities based on location in room
%ray tracing
%add metric for how long to search each part of room (multiple shelves vs
%bare floor)
%add obstacles that can be seen through but not moved through by human
%fix corner cases of victims counted as seen in q3 when looking in q2

%Draw Search Domain
% Define search domain
xMax = 80;
yMax = 40;
domainBoundaryX = [0 1 1 0 0]*xMax;  % Define search domain
domainBoundaryY = [0 0 1 1 0]*yMax;
wallT = 2;                           % Define wall thickness
exteriorX = domainBoundaryX*(xMax+2*wallT)/xMax-wallT;  
exteriorY = domainBoundaryY*(yMax+2*wallT)/yMax-wallT;  

% Draw search domain
figure(1); clf;
fill(exteriorX,exteriorY,[1 1 1]*0.6); % Space outside (gray)
hold on;
fill(domainBoundaryX,domainBoundaryY,[1 1 0.8]*1.0); % Space inside (tan)
axis equal
xlabel('X (ft)');
ylabel('Y (ft)');
xlim([-2 82]);
ylim([-2 42]);

% Position static obstacles
baseX = [1 1 -1 -1 1]; %do not touch
baseY = [1 -1 -1 1 1]; %do not touch
statTx = [12 8 8 16 2 3];  %obstacle half width
statTy = [4 6 6 2 6 10];    %obstacle half height
statX = [28 48 72 32 8 58]; %obstacle center x
statY = [4 6 6 24 28 30];    %obstacle center y
for cnt = 1:length(statX)
    fill(baseX*statTx(cnt)+statX(cnt),baseY*statTy(cnt)+statY(cnt),[1 1 1]*0.6); % Draw static obstacle
end

% Position victims about room
numVictim = 20; %default 20
inboxflag = 0;
vicCnt = 1;
while vicCnt < numVictim + 1
    inboxflag = 0;
    randPosX = floor(xMax*rand(1));   
    randPosY = floor(yMax*rand(1));
    for boxCnt = 1:length(statX)
        if inpolygon(randPosX,randPosY,baseX*statTx(boxCnt)+statX(boxCnt),baseY*statTy(boxCnt)+statY(boxCnt)) == 1
            inboxflag = 1;
        end
    end
    if inboxflag == 0
        victim(vicCnt).posx = randPosX;
        victim(vicCnt).posy = randPosY;
        victim(vicCnt).status = 1;
        plot(victim(vicCnt).posx,victim(vicCnt).posy,'bo');
        vicCnt = vicCnt + 1;
    end
end

%draw bifrication line
bifx = 16;
for bify = 8:yMax
    plot(bifx,bify,'r.')
end

%init huamns
searcher(1).type = 0; %human
searcher(1).fov = pi/3;
searcher(1).heading = 2.5;
searcher(1).posx = 60;
searcher(1).posy = 12;
for bearingInitCnt = 1:numVictim
    searcher(1).bearing(bearingInitCnt) = 0;
end

plot(searcher(1).posx,searcher(1).posy,'rx');

hL(1) = line([searcher(1).posx (searcher(1).posx + cos(searcher(1).heading-searcher(1).fov/2)*80)],[searcher(1).posy (searcher(1).posy + sin(searcher(1).heading-searcher(1).fov/2)*80)],"LineWidth",2);
hU(1) = line([searcher(1).posx (searcher(1).posx + cos(searcher(1).heading+searcher(1).fov/2)*80)],[searcher(1).posy (searcher(1).posy + sin(searcher(1).heading+searcher(1).fov/2)*80)],"LineWidth",2);

%ray tracing
for i = 1:numVictim
    %draw ray from each object to searcher
    ray(i) = line([searcher(1).posx victim(i).posx],[searcher(1).posy victim(i).posy]);
    %no easy way to calculate intersect between line and polygon, need
    %points on line
    xVec = linspace(ray(i).XData(1),ray(i).XData(2),10);
    yVec = linspace(ray(i).YData(1),ray(i).YData(2),10);
    searcher(1).bearing(i) = atan((victim(i).posy-searcher(1).posy)./(victim(i).posx-searcher(1).posx));

    %q2
    if (victim(i).posx < searcher(1).posx) && (victim(i).posy > searcher(1).posy)
        searcher(1).bearing(i) = searcher(1).bearing(i) + pi;
    end
    %q3
    if (victim(i).posx < searcher(1).posx) && (victim(i).posy < searcher(1).posy)
        searcher(1).bearing(i) = searcher(1).bearing(i) - pi;
    end
    %fix corner cases where bearing is > pi
    if searcher(1).bearing(i) > pi
        searcher(1).bearing(i) = searcher(1).bearing(i) - 2*pi;
    end
    if searcher(1).bearing(i) < -pi
        searcher(1).bearing(i) = 2*pi + searcher(1).bearing(i);
    end
    
    %mark victims with red x if outside searcher fov
    if (searcher(1).bearing(i) >= 0) && (searcher(1).bearing(i) < pi )
        if (abs(searcher(1).heading - searcher(1).bearing(i)) >= (searcher(1).fov/2))
            plot(victim(i).posx,victim(i).posy, 'ro')
            victim(i).status = 0;
            %strtxt(i) = string(searcher(1).bearing(i));
            %text(victim(i).posx,victim(i).posy, strtxt(i))
        end
    end
    if searcher(1).bearing(i) < 0
        if (abs(searcher(1).heading + searcher(1).bearing(i)) >= searcher(1).fov/2 ) 
            plot(victim(i).posx,victim(i).posy, 'ro')
            victim(i).status = 0;
        end
    end

    for j = 1:10
        for k = 1:length(statX)
            %mark victims red that are hidden behind obstacles
            if inpolygon(xVec(j),yVec(j),baseX*statTx(k)+statX(k),baseY*statTy(k)+statY(k)) == 1
                plot(victim(i).posx,victim(i).posy, 'ro')
                victim(i).status = 0;
            end                     
        end
    end
end
delete(ray);
pause(1);
% 
% delete(h);

%RRT* algorithm
stepdist = 1;
runLen = 1000;
startx = searcher(1).posx;
starty = searcher(1).posy;
nodex = zeros(1,runLen);
nodey = zeros(1,runLen);
nodeCost = zeros(1,runLen);
nodeParent = zeros(1,runLen);
xstep = zeros(1,runLen);
ystep = zeros(1,runLen);
nodex(1) = startx;
nodey(1) = starty;

for n=2:runLen
    pt = [floor(xMax*rand(1)) floor(yMax*rand(1))];
    mindist = 1000;
    %finds closest existing node to new random point
    for nodecheck=1:n-1
        dist(n) = sqrt((pt(1)-nodex(nodecheck))*(pt(1)-nodex(nodecheck)) + (pt(2)-nodey(nodecheck))*(pt(2)-nodey(nodecheck)));
        if dist(n) < mindist
            mindist = dist(n);
            bestNode = nodecheck;
            nodeCost(n) = mindist;
        end
    end
    nodeParent(n) = bestNode;
    c = n;
    p = nodeParent(c);
    while p ~= 1
        nodeCost(c) = nodeCost(c) + sqrt((nodex(c)-nodex(p))^2 + (nodey(c)-nodey(p))^2);
        c = p;
        p = nodeParent(c);
    end
            
    xstep = linspace(nodex(bestNode),pt(1),abs(ceil(nodex(bestNode)-pt(1)))+2);
    ystep = linspace(nodey(bestNode),pt(2),abs(ceil(nodey(bestNode)-pt(2)))+2);
    
    nodex(n) = xstep(2);
    nodey(n) = ystep(2);
    
    %check if point is inside any of the obstacles
    obstally = 0;
    for obscnt = 1:length(statX)
        if inpolygon(nodex(n),nodey(n),baseX*statTx(obscnt)+statX(obscnt),baseY*statTy(obscnt)+statY(obscnt)) == 0
            obstally = obstally+1;
        end
    end
    if obstally == length(statX)
        %l(n) = line([nodex(n) nodex(bestNode)],[nodey(n) nodey(bestNode)]);
    else 
        nodex(n) = 1000;
        nodey(n) = 1000;
    end
    
    %check if new node can optimize existing paths
    for v = 2:n-1
        %dist between new point and each existing point
        newdist = sqrt((nodex(v)-nodex(n))^2 + (nodey(v)-nodey(n))^2);
        %if distance to new point plus distance from new point to old point
        %is less than cost to old point
        if ((newdist + nodeCost(n)) < nodeCost(v)) && (newdist < 3) 
            nodeParent(n) = v;
            %delete(l(n));
            %l(n) = line([nodex(v) nodex(n)],[nodey(v) nodey(n)]);
            break
        end
    end
end

pause(1)

%draw trajectory to visible victims
mindist2 = 1000;
closestNode = zeros(1,numVictim);
dist2 = 1000;
for k = 1:numVictim
    %if victim is visible to searcher
    if victim(k).status ~= 0
        mindist2 = 1000;
        for m = 1:runLen
            dist2 = sqrt(((victim(k).posx - nodex(m))*(victim(k).posx - nodex(m))+((victim(k).posy - nodey(m))*(victim(k).posy - nodey(m)))));
            if dist2 < mindist2
                mindist2 = dist2; 
                closestNode(k) = m;
                parentx = nodex(closestNode(k));
                parenty = nodey(closestNode(k));
            end
        end
    end
    %trace back line of nodes from victim to searcher
    if closestNode(k) ~= 0  %closest node to each victim
        parent = nodeParent(closestNode(k));
        childx = nodex(closestNode(k));
        childy = nodey(closestNode(k));
        while parent ~= 1 
            parent = nodeParent(parent);
            childx = parentx;
            childy = parenty;
            parentx = nodex(parent);
            parenty = nodey(parent);
            line([childx parentx],[childy parenty], "LineWidth", 3, "Color", [(0.8*(k/numVictim)) 0.4 (k/numVictim)]);
            
        end
    end
             
            
    
end
elapsedTime = cputime - timestart