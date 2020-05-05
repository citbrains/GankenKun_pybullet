# GankenKun_pybullet

![GankenKun](https://user-images.githubusercontent.com/5755200/79035466-fd011080-7bf9-11ea-807a-227fc551c4ad.jpg)

### Video
https://youtu.be/kJb6OzS1FoM  
latter half: comment out `sleep`  
Computer: Core i5 + GTX1060

### main program

- GankenKun.py  
Walking control to the target position

### library for GankenKun

GankenKun/  

- kinematics.py  
Calculating the inverse kinematics based on the analytical solution  
(foot position -> joint angles)  

- foot_step_planner.py  
Calculating the footsteps from the goal position  

- preview_control.py  
Generating the trajectory of the center of mass from the footsteps based on the preview controller  

- walking.py  
Calculating the trajectory of the foot from the goal position  

### What is GankenKun

GankenKun is an open platform humanoid robot developed by CIT Brains (Chiba Institute of Technology).  
The robot had won the prize in RoboCup.  

https://github.com/citbrains/OpenPlatform

### Execute on Google Colaboratory

https://colab.research.google.com/drive/1N14EQQQPXsDvagGHN7Aw26GeYP6iAH_J?usp=sharing

### other programs (developed for understanding pybullet)  

- display.py  
Displaying the humanoid robot GankenKun

- stretch.py  
Controlling the positions of joints

- camera.py  
Capturing the camera image

- Walk.py
Checking to be able to walk (Just up and down each foot)

- inv_kine.py  
Solving the inverse kinematics

- display_COG.py
Calculating the centor of gravity

- jacobian_test.py
Calculating the Jacobian

- COG_jacobian.py
Calculating the centor of gravity Jacobian
