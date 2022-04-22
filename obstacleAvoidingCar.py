#to-do: Adjust margins according to the density of the obstacles
#to-do: Make educated emergency decision in case of obstacle closer than the length of the robot instead of a 90 degree turn

from roboticstoolbox import Bicycle, VehicleIcon, RandomPath, LandmarkMap, RangeBearingSensor
import matplotlib.pyplot as plt
from math import atan2, sqrt, pi, asin

#function to move the car with speed in angle
def move(speed, angle):
    if angle != 0:
        speed = 1
    else:
        speed = 0.5
    car.step(speed, angle)
    car._animation.update(car.x)
    plt.pause(0.005)
    return
    
def besides(distance, angle):
    if (workspace/2>distance):
        print("too close. no solution")
        return pi/4, -pi/4
    return angle + asin((workspace/2)/distance), angle - asin((workspace/2)/distance)
    

#function does it fit? takes the angle we want to go in as input at returns 0, 0 if the robot fits to go in this angle or the distance and angle of the obstacle that wouldn't make it fit
def does_fit(angle):
    #looping over all the obstacles
    for ob in sensor.h(car.x):
        d = ob[0]
        a = ob[1]
        #if the object is near enough (distance smaller than the clearance value of 5 units), check if its angle is in the current path of the car
        if d<5:
            #try-except block because d could be zero, or workspace could be greater than d, which would both generate value error
            try: 
                #find the minimum range of angles that this obstacle could obstruct (smaller distance means bigger range that could be from -90 to 90)
                ang_abs = asin((workspace/2)/d)
                if a+car.x[2]>=angle-ang_abs and a+car.x[2]<=ang_abs+angle:
                    print("Obstacle!!", d, a*180/pi)
                    return d, a
            except:
                print("problema")
                return d, a
    return 0, 0

def angle_picker(goal):
    d, a = does_fit(goal)
    if d==0 or d>distance_to_goal:
        print("going to goal")
        return goal
    else:
        paths = []
        for obstacle in sensor.h(car.x):
            if obstacle[0]<5:
                ang1, ang2 = besides(obstacle[0], obstacle[1])
                if ang1 != 0:
                    d, a = does_fit(ang1)
                    if d==0:
                        paths.append(ang1)
                    d, a = does_fit(ang2)
                    if d==0:
                        paths.append(ang2)
        ind = 0
        pre = 4
        for i in range(len(paths)):
            if abs(paths[i]-goal-car.x[2])<pre:
                pre = abs(paths[i]-goal-car.x[2])
                ind = i
        try:
            print(">>> list:",paths, paths[ind])
            return (paths[ind])
        except:
            print("Sorry, I gotta put me first. ")
            return 0

if __name__ == "__main__":
    #try:
        #Arena is 20x20m 
        grid = 20
        
        #Ask for the goal coordinates
        xf = 18
        yf = 15
        while (xf>20 or xf<0):
            xf = float(input("Please enter the x coordinate of the target to be less than 20 (grid size) and greater than zero: "))
        while (yf>20 or yf<0):
            yf = float(input("Please enter the y coordinate of the target to be less than 20 (grid size) and greater than zero: "))

        #Ask for the initial point coordinates
        x0 = 0
        y0 = 0
        while (x0>20 or x0<0):
            x0 = float(input("Please enter the car's initial x coordinate to be less than 20 (grid size) and greater than zero: "))
        while (y0>20 or y0<0):
            yf = float(input("Please enter the car's initial y coordinate to be less than 20 (grid size) and greater than zero: "))
        
        #Ask for the number of obstacles in a 20x20 Arena
        n_ob = 300
        while(n_ob<2):
            n_ob = int(input("Please enter a not-less-than-two integer number of obstacles in the 20x20 grid arena: "))
        
        #Ask for the square robot dimension (from min to max)
        robot_size = 0.8
        while (robot_size>10 and robot_size<0):
            robot_size = float(input("Please enter the square dimension of the robot from zero to 10 (default is 1): "))
            
        #the robot size plus safety margins is the robot workspace
        workspace = robot_size + 2
        
        #calculate how much should the car image be scales to equal the robot size
        scale = 1.5*robot_size/0.35
        
        #define the vehichle icon object for the robot's animation
        anim = VehicleIcon('/home/yara/CovCE/ObstacleAvoidingCar/redcar.png', scale = scale)
        #define a vehicle object of the Bicycle motion model with animation enabled in the 20x20 grid with initial coordinates as defined by the user and angle of the target
        car = Bicycle(animation=anim, control = RandomPath, dim = grid, x0 = (x0, y0, atan2(yf, xf)))
        #initialize the robot with animation
        car.init(plot = True)
        #update the animation to reflect the changes made to the default sittings in the definition line
        car._animation.update(car.x)

        #plot the goal marker on the grid
        plt.plot(xf, yf, marker='D', markersize=6, color='r')
        
        #LandmarkMap(#obstacles, from -# to +#): Make a random map with specified number of obstacles in the specified grid area
        map = LandmarkMap(n_ob, grid)
        map.plot()
        
        #define a RangebearingSensor(robot, map, animate) object 
        sensor = RangeBearingSensor(robot = car, map = map, animate = True)
        
        #trial1: obstacle avoidance
        """
        diff_x = 1
        diff_y = 1
        while abs(diff_x)>0.1 or abs(diff_y)>0.1:
            diff_x = xf - car.x[0]
            diff_y = yf - car.x[1]
            
            ang = atan2(diff_y, diff_x)
            
            distance, angle = does_fit(ang)
            if (distance == 0):
                print("here")
                #move the car in straight line (angle zero) if the current angle is the desired angle
                move(1.5, ang-car.x[2])
            else:
                break
        """
        
        #trial2: obstacle avoid
        diff_x = 1
        diff_y = 1
        while abs(diff_x)>0.5 or abs(diff_y)>0.5:
            diff_x = xf - car.x[0]
            diff_y = yf - car.x[1]
            
            distance_to_goal = sqrt(pow(diff_x, 2)+pow(diff_y, 2))
            ang = atan2(diff_y, diff_x)
            move(1.5, angle_picker(ang)-car.x[2])
        print("REACHED")

        #go_target(goal_x, goal_y)
        
        
        plt.show()
    #except:
