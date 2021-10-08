# GankenKun_pybullet

![GankenKun](https://user-images.githubusercontent.com/5755200/79035466-fd011080-7bf9-11ea-807a-227fc551c4ad.jpg)

### Video
https://youtu.be/kJb6OzS1FoM  
latter half: comment out `sleep`  
Computer: Core i5 + GTX1060

### Preparation of the development environment

Ubuntu2004
```
sudo apt install python3-pip
pip install pybullet
pip install numpy
pip install control
git clone https://github.com/citbrains/GankenKun_pybullet
cd GankenKun_pybullet
python GankenKun.py
```


[Microsoft Windows](https://github.com/citbrains/GankenKun_pybullet/wiki/%E9%96%8B%E7%99%BA%E7%92%B0%E5%A2%83%E3%81%AE%E6%BA%96%E5%82%99) (in Japanese)


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
https://github.com/citbrains/OpenPlatform_ver3_0 (now developing ver.4)  

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
