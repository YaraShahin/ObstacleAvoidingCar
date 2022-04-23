#To-do: make speed function of turning angle and the distance to target
#To-do: make margins a function of obstacle density
#Alternatevily: Uncomment line 74 (run = False) so the robot will completely stop if it must step over an obstacle (It is left commented out because the obstacle is on the robot's initial point sometimes)

#Importing code dependencies
from roboticstoolbox import Bicycle, VehicleIcon, RandomPath, LandmarkMap, RangeBearingSensor
import matplotlib.pyplot as plt
from math import atan2, sqrt, pi, asin

#function to move the car with speed in angle
def move(angle):
    car.step(1, angle)
    car._animation.update(car.x)
    plt.pause(0.005)
    return
    
#given the distance and angle of an obstacle to a robot, return the closest angles on the obstacle's left and right that will still fit the robot.
def besides(distance, angle):
    if (workspace/2>distance):
        print("too close. no solution")
        return pi/4, -pi/4
    return angle + asin((workspace/2)/distance), angle - asin((workspace/2)/distance)
    

#function does it fit? takes the angle we want to go in as input at returns 0 if the robot fits to go in this angle or the distance of the obstacle that wouldn't make it fit
#For each obstacle, check if near enough (distance smaller than the clearance value of 5 units), and its angle is in the current path of the car
def does_fit(angle):
    for ob in sensor.h(car.x):
        if ob[0]<5:
            #try-except block because ob[0] could be zero, or workspace could be greater than d, which would both generate value error
            try: 
                #find the minimum range of angles that this obstacle could obstruct (smaller distance means bigger range that could be from -90 to 90)
                ang_abs = asin((workspace/2)/ob[0])
                if ob[1]+car.x[2]>=angle-ang_abs and ob[1]+car.x[2]<=ang_abs+angle:
                    print("Obstacle!!")
                    return ob[0]
            except:
                print("Value Error. The obstacle is too close.")
                return ob[0]
    return 0

def angle_picker(distance_to_goal, angle_to_goal):
    global run
    
    #get the distance of the obstacle in the target's angle. 
    distance_to_obstacle = does_fit(angle_to_goal)
    #If there's no obstacle (distance 0), or the target is closer than the obstacle, go to goal
    if distance_to_obstacle==0 or distance_to_obstacle>distance_to_goal:
        print("going to goal")
        return angle_to_goal
    #Else, loop on all obstacles and see the closest angle left and right of the near obstacles that will still fit the robot
    else:
        paths = []
        for obstacle in sensor.h(car.x):
            if obstacle[0]<10:
                ang1, ang2 = besides(obstacle[0], obstacle[1])
                if ang1 != 0:
                    if does_fit(ang1)==0:
                        paths.append(ang1)
                    if does_fit(ang2)==0:
                        paths.append(ang2)
        #from all these angles in list paths, choose the one that is closest to the target. If the length of the list is zero (no routes available), stop the car by setting the flag to false
        ind = 0
        pre = 4
        for i in range(len(paths)):
            if abs(paths[i]-angle_to_goal-car.x[2])<pre:
                pre = abs(paths[i]-angle_to_goal-car.x[2])
                ind = i
        try:
            #print(">>> list:",paths, paths[ind])
            return (paths[ind])
        except:
            print("Sorry, I gotta put me first. ")
            #run = False
            return angle_to_goal

if __name__ == "__main__":
    try:
        #Arena is 20x20m 
        grid = 20
        
        #Ask for the goal coordinates in the allowed range of grid
        xf = -21
        yf = -21
        while (abs(xf)>20):
            xf = float(input("Please enter the x coordinate of the target to be between -20 and 20 (default 18): "))
        while (abs(yf)>20):
            yf = float(input("Please enter the y coordinate of the target to be between -20 and 20 (default 15): "))

        #Ask for the initial point coordinates in the allowed range of grid
        x0 = -21
        y0 = -21
        while (abs(x0)>20):
            x0 = float(input("Please enter the car's initial x coordinate between -20 and 20 (default zero): "))
        while (abs(y0)>20):
            y0 = float(input("Please enter the car's initial y coordinate between -20 and 20 (default zero): "))
        
        #Ask for the number of obstacles in a 20x20 Arena
        n_ob = 0
        while(n_ob<2):
            n_ob = int(input("Please enter a not-less-than-two integer number of obstacles in the 20x20 grid arena (default 80): "))
        
        #Ask for the square robot dimension (from min to max)
        robot_size = 0.8
        while (robot_size>=3 and robot_size<0):
            print("Please check documentation for robot size vs. obstacle density to ensure robustness.")
            robot_size = float(input("Please enter the square dimension of the robot from zero to 3 (default is 0.8): "))
            
        #the robot size plus safety margins is the robot workspace
        workspace = robot_size + 1
        
        #calculate how much should the car image be scales to equal the robot size
        scale = 1.5*robot_size/0.35
        
        #Ask the user for the image name
        img_path = input("Please enter the path of the image you want to use, or 0 for MY default: ")
        if(img_path=="0"):
            img_path = "/home/yara/CovCE/ObstacleAvoidingCar/redcar.png"
        
        #define the vehichle icon object for the robot's animation
        anim = VehicleIcon(img_path, scale = scale)
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
        
        #Obstacle avoidance algorithm
        #run flag that falsifies only if no solutions are found and the robot must stop for safety
        run = True
        while True:
            x_to_goal = xf - car.x[0]
            y_to_goal = yf - car.x[1]
            
            #the loop breaks if no solutions are found, the goal is reached, or the robot is out of bounds.
            if( not run):
                print("No solution! Please decrease the number of the obstacles or the size of the robot.")
                break
            elif(abs(x_to_goal)<=0.5 and abs(y_to_goal)<=0.5):
                print("I actually did it! I reached the target!")
                break
            elif (abs(car.x[0])>25 or abs(car.x[1])>25):
                print("So sorry, but I'm outta here.")
                break
                
            #Calculate the distance and the angle to the goal
            distance_to_goal = sqrt(pow(x_to_goal, 2)+pow(y_to_goal, 2))
            angle_to_goal = atan2(y_to_goal, x_to_goal)
            
            #move the robot in the angle chosen by the angle picker
            move(angle_picker(distance_to_goal, angle_to_goal)-car.x[2])

        plt.show()
    except:
        print("So sorry, there's an unexpected error somehow? Can you pass me the laptop so I can double-check?")

