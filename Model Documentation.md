# Highway Driving - Path Planning

# Localization
The localization of the ego vehicle is out of scope of this project and it is 
only expected that the localized cartesian and frenet coordinates of the ego 
vehicle are updated properly.

# Prediction
The sensor fusion data is updated every cycle which contains the detected 
obstacles/objects. The implementation keeps track of the objects in front and 
object from behind of the ego vehicle for every valid lane. Based on these 
environmental information, the implemetation makes a decision on whether to 
continue on the same lane or change lane and hands it over to the controller
in the form of a predicted trajectory which is just the car’s s-position 
(in Frenet-coordinates) on the target lane. We use here an assumption, that 
the relative positions of the vehicles on the track do not change very much 
within the prediction horizon.This assumption does not hold in real traffic, 
but turned out to be sufficient in the simulated environment. The objects nearer
to the ego vehicle is updated in the function `Vehicle::update_nearby_objects` 
in `vehicle.cpp`

# Behaviour Planning

The behaviour planner is a finite state machine with the following states

    INIT
    KEEP LANE
    PREPARE LANE CHANGE TO RIGHT
    CHANGE LANE RIGHT
    PREPARE LANE CHANGE TO LEFT
    CHANGE LANE LEFT 

The states are mostly self-explanatory. INIT state is for accelerating from a
stand still position to highway speed. This state uses different acceleration 
profile than other states. Based on the transition states, different maneuvers 
are created which have the desired lane and the target speed.

Cost Estimation

Cost::calculate_cost() estimates cost for each state maneuver. The cost is a 
combination of the following sub-costs:

    Collision Cost: This cost penalises the collision to another vehicle on the
                    target lane.There is also a smaller penalty, if another vehicle 
                    is close, but not actually colliding.
    Lane speed cost: This cost calculates the difference between the speed 
                     limit and current vehicle in the front of the ego vehicle. 
                     The penalty is directly proportional to the magnitude of the difference.
    Free lane cost: This cost rewards the free lane space in the front (i.e. 
                    how far the next car in front is) and the reward is directly
                    proportional to the length of the free space. Maximum reward is given 
                    when there is 75.0 meters or more free lane ahead.
    Keep lane cost: There is a small reward for keeping the ego lane. The purpose of this 
                    term is to prevent the car swaying between lanes when the other cost 
                    terms give approximately equal result for two adjacent lanes, and not to 
                    change lanes when the conditions are equal.

The state maneuver with lowest cost is chosen as the target maneuver. The next 
step is to generate a trajectory for the chosen maneuver.

# Trajectory

Vehicle’s trajectory is generated based on the chosen maneuver from the behaviour 
planner which has both the target lane and target speed. The generated trajectory
consist of 50 points in the global cartersian coordinates. The simulator consumes 
one point every 20 ms. This means, that the horizon of the trajectory is one second, 
and that the speed and acceleration of the vehicle is determined by the spacing of the points.
On each cyc;e, the trajectory generator reuses ten points of the previous trajectory.
This is done in order to provide a smooth transition from the old trajectory to the new one. 
It also means, that the vehicle has 0.2 seconds ‘reaction time’.

We use a spline-library to generate the trajectory curve. The base point for the spline is 
the last reused point (at the beginning, car’s position). For end points the generator uses 
Frenet coordinates with s-coordinate 40 and 80 meters away from the base point. 
The d-coordinate of these points is the centre of the target lane. These reference points are
 converted to vehicle’s cartesian coordinate space (the base point as origin) before fitting
to a spline. For the final trajectory, car coordinates are transformed to global cartesian (x,y) 
space.
