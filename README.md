# searchAndRescueSimulation

Introduction:
	The goal of this exercise was to model the search behavior of a mixed team consisting of both Human and Robotic rescuers in a firefighting operation in order to develop insight into the effectiveness of different search strategies. With the assumption that the models of search and movement capabilities of both human and robot are sufficiently close to their real life counterparts, strategies found to be successful in the simulation should be applicable to testing in a lab setting. The simulation described below was devised before an in class mock rescue operation, the results of which were used to help verify a chosen search strategy.

Methods:
	Several assumptions were made in order to make simulating the operation possible. 
Firstly, the entire simulation was represented in two dimensional space. While this assumption is logical for a rescue operation taking place on a single floor of a building, this program would not be suitable for searching multiple floors. 
A searcher’s Field Of View (FOV) was represented as the area between two rays extending from the searcher’s center of mass at a set angle. Any object with a bearing outside the FOV would be unable to be seen. At each timestep, a simple ray tracing operating was performed for each searcher. Any objects within a searcher FOV had a line drawn between their location and the location of their respective searcher(s). For each ray, 10 equidistant points were generated and tested to see if they fell inside an obstacle, meaning that the direct line of sight between searcher and victim was blocked. All obstacles in the environment, with the exception of the collapsing floor, were set as opaque to searchers, and would completely obscure objects hidden behind them. In the figure below, blue circles represent the subset of victims that are visible given the searcher position and FOV. This process was made into a function searchForVictim(). 


At a given time step if there was a direct line of sight and the victim was within a searcher’s field of view, the chance of a searcher finding a potentially visible victim was modeled as proportional to the distance from the searcher. A potentially visible victim 1 unit away or less from the searcher would be guaranteed to be seen, while a vicitim n feet away would have a probability 1/n chance of being found for each time step. It was assumed that victims were spread randomly throughout the search space.
In one part of an NIH study seeking to determine metrics for firefighter endurance in full Personal Protective Equipment (PPE), subjects were asked to maintain various paces on a treadmill until exhaustion. Given that the firefighters needed to be inside the search space in our simulation for only 80 seconds, even with a factor of safety of 500% the searchers should only need to be able to last six and a half minutes of walking. In one experiment, subjects with full PPE were able to maintain a modest pace of 5.3 km/hr (around 5ft/s) with a group mean for time to exhaustion of 7 ± 1.5 min1. Thus, 5 ft/s was determined to be the maximum safe speed. Continuing with the assumption that rescuers can more easily perform a careful search of the space when not moving at close to their maximum safe speed, human searcher speed for the simulation was set as 75% of max speed or 3.75 ft/sec. 

It was assumed that all searchers were able to maintain a perfect level of communication- that is to say that once one searcher identified the location of a victim, all other search agents would also have access to the location of that victim. In order to plan a path between a searcher and a known victim, a Rapidly-expanding Random Tree (RRT) search was initially devised. This algorithm begins with a single node at the starting point of the searcher and generates a random point in the search space that does not sit within any obstacle. A line segment of length 1 is then generated from the closest existing node to the new point in the direction of the new point. A new node is then created at the end of this “branch” segment and the process is repeated. Throughout the process an array is generated that records the parent node of each new node. Once a node is within a critical distance of a victim (represented by a circle of radius 1 surrounding the victim), the path is assumed to have reached the victim and is traced back to the starting point using the parent array. Given a sufficiently large number of iterations, this algorithm will approach every reachable point in the search space with probability 1.  This strategy of path planning does not take into account the overall length of the path from searcher to victim.



Slight modifications were made to previous strategy to create an RRT* search, which asymptotically approaches the most optimal trajectory, at the cost of a longer solution time2. In addition to storing information on the parent of each node, an additional matrix was generated to store the cost of reaching each node. In this case, the cost of a node is defined as the total distance traveled to reach that node. After each iteration of the search process,  the algorithm checks if the most recently added node N can optimize any existing paths. If the cost to the new node plus the distance between the new node and a point A is less than the previous cost to A, then N becomes the parent node of point A and the cost of A is then updated to the cost of N + the distance between N and A.  Once the tree is generated, the distance between each node and the goal position is calculated (in this example a located victim) and the node with the least distance from the target is the final node. Using the parent matrix, the path of nodes is followed back from the goal position to the starting position, plotting each sub segment along the way. This process was made into the function rrtpath() so that it could be scaled and used with multiple search agents simultaneously.   



	In order to overcome the issue of different paths for the two searchers not necessarily having the same number of steps, a third function stepForward() was generated that handled the movement of each rescuer. Called on every iteration of the main loop, stepForward() would move each searcher one node closer to the goal node if not already at the end of a branch, and would generate a new path for the searcher using rrtpath() when the searcher had arrived at the goal node. Regardless if stepForward() causes the searcher to step forward on an existing path or take the first step on a new path, the searchForVictim() function will be called and executed at each step, meaning that each agent is continuously searching the area as it moves along.
	Because the distance between connected nodes ~1 (new nodes are placed exactly 1 ft away from the nearest existing node, with new connections that result in optimizing the existing tree resulting in slightly shorter connections) and an average walking speed of 3.75 ft/sec, the simulation is set to run for 250 timesteps for each searcher so that regardless of their position, a searcher has enough time to escape once their time is up. Searchers begin at the entry point (60,0) and have an initial target point set in the room. Searchers move towards their respective target point, calling the searchForVictim() function at each step along the way. In the visualization, each victim is represented as a red dot, and once seen the victim is marked with a green circle surrounding their location. Once a searcher reaches their initial goal, their next target is set as the closest located victim and a variable is set indicating their target such that the two searchers will not attempt to rescue the same victim. Upon reaching a victim, the victim’s status is set safe and their dot is removed from the graph. Each searcher has a variable that shows the total number of victims that they are currently carrying. Once a searcher is carrying 5 (the max number of victims), their goal point is set to the entry point and they will traverse to the exit, and their count of victims being carried is set to 0 once the exit is reached. The searcher then continues searching for victims until their step count reaches 250.	
	


	Searchers used an additional step count for each step they were inside the polygon representing the burning section of the building. The flame front used in this simulation followed the equations where xw represents the top of the front and xc represents the bottom of the wall of fire progressing across the room from left to right:

xw(k) = xw(k-1) + v*dt + n(k)
xc(k) =  xc(k-1) + v*dt + u(k)

	In the live run of the searching experiment in the Tufts CRISP Lab it was determined that the drone’s utility was limited to identifying hazards within the search space (collapsing floor) due to a combination of low camera resolution and minimal pilot training. For that reason, in simulation the drone was simplified to a random variable that would set the collapsing floor to one of two locations. In order to account for the time taken for the drone to reach the hazard, the simulation begins with the flame front already at k=30 representing a time of 10 seconds has elapsed. Unlike other obstacles in the search space, the collapsing floor would not be accounted for in the ray tracing operation of searchForVictim() so that searchers would be able to see through the hazard but not traverse directly over it.


Results:
	Two main cases were tested in this experiment. Case 1, where both searchers would enter from the starting location and then head straight to the back of the room (x = 0) and go through their search indiscriminately choosing to explore regions both inside and outside the flame front, and Case 2 where both searchers would enter towards the back of the room but this time if no victims are found favor randomly searching in locations that had not yet been reached by the flames. The goal was to determine if it is a better strategy to try and save as many victims as possible early on, or to save resources in order to give the searchers as much time as possible in the search space at the cost of inadvertently favoring victims one side of the room. A Monte Carlo simulation was constructed and ran for 25 trials of each case.



Mean Victims Rescued
Standard Deviation
Min
Max
Case 1
12.48
1.73
8
16
Case 2
10.52
2.64
5
15





Discussion:
Qualitatively, it is worth noting that for a large number of the trials in Case II, the searcher entirely avoided the dead end section of the search space leading up to the region with the collapsed floor. Because the searcher is setting its random trajectory outside of the area of flames, by the time that the searcher has reached the initial goal position and retrieved victims along the way, it is relatively unlikely that searcher will choose to randomly visit an area in the dead end section of the room before it is engulfed in flames. Additionally, in Case II, if a seacher does not immediately locate any victims in the leftmost portion of the room from x = 0 to x =16 on the first pass, they will likely not get saved later on in the simulation. While Case II excels in situations where a large number of victims are closely clustered in the right half of the room, this is usually not the case and the strategy is less effective overall. Because of this inherent variability, Case 2 resulted in a substantially higher standard deviation than Case 1. 
	In either case, there was a strong positive relationship between the density of clusters of victims and the probability of finding victims in the cluster. Often, a searcher would initially locate just one victim in the cluster but would discover more as the searcher began to move closer to retrieve the first identified victim. 
	In the real world simulation in the CRISP Lab, the number of victims rescued throughout the five successful trials (ignoring the first trial in which the rescuer timing was not set correctly) 	ranged from a minimum of 8 to a maximum of 15. These results are similar to the min and max values found in simulation.

Summary:

	The results of this simulation suggest that the best strategy for finding as many victims as possible given the constraints of limited search time and additional time penalty for each step a searcher is inside the flame front, is to ignore the flame front when planning a trajectory and continue to search for areas already passed by the fire at additional cost. In this simulation, the cost of double oxygen expenditure was not sufficiently high enough to overcome the potential benefit of covering a larger amount of the search space. Had the oxygen cost of being inside the flame front been higher, it is likely that paths venturing too far into the flames would allow much less space to be covered (and thus fewer victims found) however, a 200% oxygen consumption rate was not a large enough cost for that to be the case. 
Alternatively, had the probability of finding a victim in each time step calculated by the searchForVictim() function been substantially lower, it is possible that Case 2 would have given superior results. Because there is a reasonably high likelihood of finding a victim if the searcher is less than 2 feet away, it seems that the searchers in the current version of Case 2 quickly exhaust all of the findable victims in the rightmost pocket of the search space and then waste up to 100 time steps looking around there while there are plenty of unfound victims beyond the flame front that the search agents will not attempt to find. Making the probability of finding a victim more difficult will lower the overall success rate across the two cases but reduces the “wasted” time spent searching in the upper right section of the room for Case 2 making the strategy relatively more viable. That being said, the “real world” searches resulted in similar number of victims being located as in the control case of the simulation, so it can be assumed that the probability of a searcher finding victims was somewhat reasonable for the conditions of the lab search. It is important to note that results from this simulation should be taken with caution, as there were numerous simplifications that may limit the scope to which the solution can be applied. Further improvements to simulation robustness such as verification of search probabilities as well as more accurate modeling of agent communication (or lack thereof) are required before safely being able to apply this to a real world rescue operation.



References:

Lee, Joo-Young, et al. “The Impact of Firefighter Personal Protective Equipment and Treadmill Protocol on Maximal Oxygen Uptake.” Journal of Occupational and Environmental Hygiene, vol. 10, no. 7, 2013, pp. 397–407., doi:10.1080/15459624.2013.792681.
Karaman, Sertac, and Emilio Frazzoli. “Sampling-Based Algorithms for Optimal Motion Planning.” Cornell University, 2011, doi:1105.1186.
