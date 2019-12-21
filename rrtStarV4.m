%Matt McDermott
%ME149: Colaborative Robotics Final Simulation

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

%drone spots where the collapsing floor is
droneRand = rand(1);
if droneRand > 0.5
    barrier = 15;
else
    barrier = 33;
end

% Position static obstacles
baseX = [1 1 -1 -1 1]; %do not touch
baseY = [1 -1 -1 1 1]; %do not touch
statTx = [12 8 8 16 2 3 3];  %obstacle half width
statTy = [4 6 6 2 6 10 7];    %obstacle half height
statX = [28 48 72 32 8 58 19]; %obstacle center x 
statY = [4 6 6 24 28 30 barrier];    %obstacle center y 15 or 33
for cnt = 1:length(statX-1)
    fill(baseX*statTx(cnt)+statX(cnt),baseY*statTy(cnt)+statY(cnt),[1 1 1]*0.6); % Draw static obstacle
end
fill(baseX*statTx(length(statX))+statX(length(statX)),baseY*statTy(length(statX))+statY(length(statX)),[1 0.5 0.5]*0.6)

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
        victim(vicCnt).status = 0;
        vp(vicCnt) = plot(victim(vicCnt).posx,victim(vicCnt).posy,'r.','MarkerSize',30);
        vicCnt = vicCnt + 1;
    end
end

%draw bifrication line
bifx = 16;
for bify = 8:yMax
    plot(bifx,bify,'r.')
end

%init huamns
for w = 1:2
searcher(w).type = 0; %human
searcher(w).fov = pi/3;
searcher(w).heading = pi/4;
searcher(w).posx = 1;
searcher(w).posy = 1;
atTarget(w) = 0;
for bearingInitCnt = 1:numVictim
    searcher(w).bearing(bearingInitCnt) = 0;
end
hL(w) = line([searcher(w).posx (searcher(w).posx + cos(searcher(w).heading-searcher(w).fov/2)*80)],[searcher(w).posy (searcher(w).posy + sin(searcher(w).heading-searcher(w).fov/2)*80)],"LineWidth",4);
hU(w) = line([searcher(w).posx (searcher(w).posx + cos(searcher(w).heading+searcher(w).fov/2)*80)],[searcher(w).posy (searcher(w).posy + sin(searcher(w).heading+searcher(w).fov/2)*80)],"LineWidth",4);
end

%main----------------------------------------------------------------------
%enter room and walk to opposite side to start, looking for victims
xstart = 60;
ystart = 2;
xend(1) = 10; %setpoint for initial movement- place in center of room for best results
yend(1) = 5;
xend(2) = 5;
yend(2) = 37;
[X1, Y1] = rrtpath(xstart,ystart,xend(1),yend(1),10000, barrier);
[X2, Y2] = rrtpath(xstart,ystart,xend(2),yend(2),10000, barrier);
steps(1) = 1;
steps(2) = 1;
stepsTally(1) = 0;
stepsTally(2) = 0;
stepsTotal(1) = 0;
stepsTotal(2) = 0;
carrying(1) = 0;
carrying(2) = 0;
randomTraj(1) = 1; %first trajectory is not going to victim, dont increase carry count
randomTraj(2) = 1;

flameb = -10; %starting x for top and bottom of flamefront
flamet = -10; %start y
lastHead(1) = pi/2;
lastHead(2) = pi/2;

%stepsize <= 1ft. Avg walking speed of searcher = 5ft/sec therefore 5*80 =
%400, leaving time to escape, stepsTotal = 300

while (stepsTotal(1) < 250) && (stepsTotal(2) < 250)
%traverse from xstart,ystart to xend,yend
    for s = 1:2
       if stepsTally(s) < steps(s)
           stepsTally(s) = steps(s);
       else
           stepsTotal(s) = stepsTally(s) + stepsTotal(s)
           stepsTally(s) = 0;
       end
       %move flamefront
       flameb = flameb + 0.2 + 0.3*randn(1); %bottom of flamefront
       flamet = flamet + 0.2 + 0.3*randn(1); %top
       flamefront = polyshape([-2 flameb flamet -2],[-2 -2 42 42]);
       fire = plot(flamefront);
       %update searcher pos
       if atTarget(s) == 1
           stepsTotal(s) = stepsTotal(s) + steps(s);
           if s == 1
                [X1, Y1] = rrtpath(searcher(s).posx,searcher(s).posy,xgoal(s),ygoal(s),10000, barrier)
                steps(s) = 1;
                atTarget(s) = 0;
                lastHead(1) = 0;
           end
           if s == 2
                [X2, Y2] = rrtpath(searcher(2).posx,searcher(2).posy,xgoal(s),ygoal(s),10000, barrier)
                steps(s) = 1;
                atTarget(s) = 0;
                lastHead(2) = 0;
           end
       end       
       if s == 1
           [xout, yout, steps(1), atTarget(1), lastHead(1), hL(1), hU(1)] = stepForward(X1,Y1,steps(1),s,lastHead(1),hL(1),hU(1));
            searcher(s).posx = xout;
            searcher(s).posy = yout;
       end 
       if s == 2
           [xout, yout, steps(2), atTarget(2), lastHead(2),  hL(2), hU(2)] = stepForward(X2,Y2,steps(2),s,lastHead(2),hL(2),hU(2));
           searcher(s).posx = xout;
           searcher(s).posy = yout;
       end      
       carryStr(s) = string(carrying(s));
              
       %plot searcher COM pos       
       p1(s,steps(s)) = plot(searcher(s).posx,searcher(s).posy,'b.','MarkerSize',30);
       carryNum(s) = text(searcher(s).posx,searcher(s).posy,carryStr(s),'FontSize',30);
       
       %deplete steps faster if inside flamefront
       if inpolygon(searcher(s).posx,searcher(s).posy,[-2 flameb flamet -2],[-2 -2 42 42]) == 1
           %steps(s) = steps(s) + 1; %need replacement variable
       end
       
       victim = searchForVictim(searcher(s).posx, searcher(s).posy, searcher(s).heading, searcher(s).fov, victim);
       for v = 1:numVictim
           if victim(v).status == 1
               vpf(v) = plot(victim(v).posx,victim(v).posy, 'go','MarkerSize',50);
           end
           if victim(v).status == 2
               delete(vp(v))
           end
       end

       if s == 2
           pause(0.0625);
           delete(p1);
           %delete(hL);
           %delete(hU);
           delete(carryNum); %text displaying how many victims being carried
           
       end
       delete(fire);
       %end of trajectry
       if (((steps(s) == length(X1)-1)&& s == 1)||((steps(s) == length(X2)-1) && s == 2)) && (randomTraj(s) == 0)
           victim(closestVic(s)).status = 2;
           carrying(s) = carrying(s) + 1;
           stepsTotal(s) = stepsTotal(s) + 5; %takes 5s on avg to pick up victim
       end
       if  (atTarget(s) == 1) && (searcher(s).status == 1) && (searcher(s).posx < 62) && (searcher(s).posx > 58)
           carrying(s) = 0;
           %atTarget(s) = 0;
           searcher(s).status = 0;
           xgoal(s) = (xMax-flameb)*rand(1)+flameb;
           ygoal(s) = yMax*rand(1);
           randomTraj = 1;
       end
       %steps(s) = steps(s) + 1
    
    delete(vp)

    shortest(s) = 1000;
    closestVic(s) = 1;
    vicHiddenCnt = 0;
    for m = 1:numVictim
        if (victim(m).status == 0) || (victim(m).status == 1)
            vp(m) = plot(victim(m).posx,victim(m).posy, 'r.','MarkerSize',30);
        end
        %get count of victims unseen or seen and found
        if (victim(m).status == 0) || (victim(m).status == 2)
            vicHiddenCnt = vicHiddenCnt + 1;
        end
        if victim(m).status == 1
            distTo = sqrt((searcher(s).posx - victim(m).posx)^2 + (searcher(s).posy-victim(m).posy)^2);
            if distTo < shortest(s)
                shortest(s) = distTo;
                closestVic(s) = m;
            end
        end
        if victim(m).status == 2 %found and reached
            delete(vp(m))
        end
        
    end
    %if no victims seen but not rescued, go to random location
    if vicHiddenCnt == numVictim
        %xgoal(s) = (xMax-flameb-)*rand(1)+flameb;
        xgoal(s) = 10*rand(1) + flameb;
        %xgoal(s) = xMax*rand(1);
        ygoal(s) = yMax*rand(1);
        randomTraj(s) = 1;
    else
        xgoal(s) = victim(closestVic(s)).posx;
        ygoal(s) = victim(closestVic(s)).posy;
        randomTraj(s) = 0;
    end
    %once carrying 3 victims, exit fire
    if carrying(s) == 3
        xgoal(s) = 60;
        ygoal(s) = 1;
        searcher(s).status = 1; %leaving with full load
        randomTraj(s) = 1;
    end
    end
    
end
%--------------------------------------------------------------------------

function [victim] = searchForVictim(sx,sy,sh,fov,victim)
numVictim = 20;
xMax = 80;
yMax = 40;
domainBoundaryX = [0 1 1 0 0]*xMax;  % Define search domain
domainBoundaryY = [0 0 1 1 0]*yMax;
wallT = 2;                           % Define wall thickness
exteriorX = domainBoundaryX*(xMax+2*wallT)/xMax-wallT;  
exteriorY = domainBoundaryY*(yMax+2*wallT)/yMax-wallT; 
baseX = [1 1 -1 -1 1]; %do not touch
baseY = [1 -1 -1 1 1]; %do not touch
statTx = [12 8 8 16 2 3];  %obstacle half width
statTy = [4 6 6 2 6 10];    %obstacle half height
statX = [28 48 72 32 8 58]; %obstacle center x
statY = [4 6 6 24 28 30];    %obstacle center y
for i = 1:numVictim
    %draw ray from each object to searcher
    ray(i) = line([sx victim(i).posx],[sy victim(i).posy]);
    %no easy way to calculate intersect between line and polygon, need
    %points on line
    xVec = linspace(ray(i).XData(1),ray(i).XData(2),10);
    yVec = linspace(ray(i).YData(1),ray(i).YData(2),10);
    searcher(1).bearing(i) = atan2((victim(i).posy-sy),(victim(i).posx-sx));
    %chance of finding victim at a given timestep is proportioal to
    %distancee from searcher
    dist = sqrt((sx - victim(i).posx)^2 + (sy-victim(i).posy)^2);
    findChance = dist*rand(1);
    
    %mark victims with blue o if inside searcher fov and not obscured 
    if (abs(sh - searcher(1).bearing(i)) <= (fov/2))  && (findChance < 1) && (victim(i).status ~= 2)    
        m = 1;
        for j = 1:10 %breaks up each ray into 10 points, checks if each point is in obstacle
            for k = 1:length(statX)
                %mark victims red that are hidden behind obstacles
                if inpolygon(xVec(j),yVec(j),baseX*statTx(k)+statX(k),baseY*statTy(k)+statY(k)) == 0
                    m = m + 1;
                end                     
            end
        end
        %victim is not blocked by any obstacle
        if m == 10*length(statX)+1
            victim(i).status = 1;
        end
    end    
    delete(ray);
end
end
% 
% delete(h);

function [pathx, pathy] = rrtpath(xstart,ystart,xend,yend,fidelity, barrier)
%RRT* algorithm
stepdist = 1;
runLen = fidelity;
nodex = zeros(1,runLen);
nodey = zeros(1,runLen);
nodeCost = zeros(1,runLen);
nodeParent = zeros(1,runLen);
xstep = zeros(1,runLen);
ystep = zeros(1,runLen);
nodex(1) = xstart;
nodey(1) = ystart;

xMax = 80;
yMax = 40;
domainBoundaryX = [0 1 1 0 0]*xMax;  % Define search domain
domainBoundaryY = [0 0 1 1 0]*yMax;
wallT = 2;                           % Define wall thickness
exteriorX = domainBoundaryX*(xMax+2*wallT)/xMax-wallT;  
exteriorY = domainBoundaryY*(yMax+2*wallT)/yMax-wallT; 
baseX = [1 1 -1 -1 1]; %do not touch
baseY = [1 -1 -1 1 1]; %do not touch
statTx = [12 8 8 16 2 3 3];  %obstacle half width
statTy = [4 6 6 2 6 10 7];    %obstacle half height
statX = [28 48 72 32 8 58 19]; %obstacle center x
statY = [4 6 6 24 28 30 barrier];    %obstacle center y


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

%draw trajectory to xend yend
mindist2 = 1000;
closestNode = 0;
dist2 = 1000;
mindist2 = 1000;
for m = 1:runLen
    dist2 = sqrt(((xend - nodex(m))*(xend - nodex(m))+((yend - nodey(m))*(yend - nodey(m)))));
    if dist2 < mindist2
        mindist2 = dist2; 
        closestNode = m;
        parentx = nodex(closestNode);
        parenty = nodey(closestNode);
    end
end
%trace back line of nodes from victim to searcher
if closestNode ~= 0  %closest node
    parent = nodeParent(closestNode);
    childx = nodex(closestNode);
    childy = nodey(closestNode);
    count = 1;
    while parent ~= 1 
        parent = nodeParent(parent);
        childx = parentx;
        childy = parenty;
        parentx = nodex(parent);
        parenty = nodey(parent);
        pathx(count) = childx;
        pathy(count) = childy;
        count = count + 1;
        %display trajectory line -------
        line([childx parentx],[childy parenty], "LineWidth", 3, "Color", [0.8 0.2 1]);

    end
    
end
%searcher start pos -> goal point
pathx = flip(pathx);
pathy = flip(pathy);
end

function [xout, yout, t, atTarget, lastHead, hL, hU] = stepForward(X,Y,t,s,lastHead,hL,hU)
    
    
    %moves searcher one timestep forwards
    if t < length(X)-1
        xout = X(t);
        yout = Y(t); 
        atTarget = 0;
        searcher(s).posx = X(t);
        searcher(s).posy = Y(t);
        searcher(s).fov = pi/3;
    else 
        atTarget = 1;
        xout = X(t);
        yout = Y(t);
        searcher(s).posx = X(t);
        searcher(s).posy = Y(t);
        t = 1;
        searcher(s).fov = pi/3;
        searcher(s).heading = 0;
    end
    t = t + 1;
    %update searcher heading
       if (t > 1) && (length(Y)>1) && (length(X) > 1)
            searcher(s).heading = atan2((Y(t)-Y(t-1)),(X(t)-X(t-1)));
            delete(hL)
            delete(hU)
       end
       %smooth out rapid changes in heading due to sharp turns
       if (t > 2) && (abs(searcher(s).heading - lastHead) < pi/2)
           searcher(s).heading = (searcher(s).heading + lastHead)/2;
       end
       
       lastHead = searcher(s).heading;
       %draw fov lines
       hL = line([searcher(s).posx (searcher(s).posx + cos(searcher(s).heading-searcher(s).fov/2)*80)],[searcher(s).posy (searcher(s).posy + sin(searcher(s).heading-searcher(s).fov/2)*80)],"LineWidth",4);
       hU = line([searcher(s).posx (searcher(s).posx + cos(searcher(s).heading+searcher(s).fov/2)*80)],[searcher(s).posy (searcher(s).posy + sin(searcher(s).heading+searcher(s).fov/2)*80)],"LineWidth",4);
end
            
