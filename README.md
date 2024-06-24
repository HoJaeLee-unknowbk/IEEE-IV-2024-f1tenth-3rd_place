Before doing F1tenth_Korea 2nd, we received the FGM code at Boot_camp held at the National Gyeongsang National University and used it to participate in the F1tenth_Korea 2nd competition.
However, the FGM code did not take a shortcut and we could not climb the podium.

The FGM code was very stable and good to speed up. So we used FGM as it is, but we changed the code so that we could use the shortcut using Localization.
Thanks to this, we finished 3rd place in the IEEE IV 2024 F1tenth Grand Prix.

The vehicle basically moved using the Follow_The_Gap algorithm, and when entering a specific area, it headed to the Goalpoint using the Pure_Persuit method.
The following code specifies the Pure_Pursuit formula, Goal_point, and a specific area.

        xgoal= 3.840
        ygoal=-4.696
        a=math.atan((xgoal-self.current_odom_x)/(ygoal-self.current_odom_y))
        LFD=1.9
        image_angle=math.atan((2*0.25*math.sin(a))/LFD)
        true_angle=image_angle

        if 0.508<= self.current_odom_x <=4.303 and -6.556 <= self.current_odom_y <= -4.819:
          velocity=5.0
          angle=true_angle
          print(true_angle)

First:goal_point

In the method of obtaining coordinates in a map, you can obtain coordinates in one place by typing the code below on the terminal and performing 2d pose estimation with RVIZ.

ros2 topic echo /initialize

So, you can specify the desired Goal_point and enter xgoal and ygoal.

Second:specific area

In order to specify the desired area, you can set the area by obtaining coordinates at the top and bottom of the rectangle through initial pose.

Therefore, if you enter the code below according to the conditions, a rectangle will be created, and it will become a specific area.

if 0.508<= self.current_odom_x <=4.303 and -6.556 <= self.current_odom_y <= -4.819:
          velocity=5.0

Third:LFD

When a vehicle enters a specific area, it should tune the LFD and speed to see if it goes to the desired Goal_Point. LFD is a Look Forward Distance and Pure_Pursuit formula that corresponds to Look Ahead, and the larger the size, the slower the vehicle's curve runs.


We decided on two specific areas, through which we overcame steep corners and shortcuts.

![jeju_area](https://github.com/HoJaeLee-unknowbk/IEEE-IV-2024-f1tenth/assets/172498133/28aa18ca-7bc7-40a7-a87e-61597e2804dc)


