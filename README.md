# Controls-PID
Self-Driving Car Engineer Nanodegree Program

For installation, please read `Dependencies.md`


# Steering controller
Steering control is implemented in `src/main.cpp` which uses `pid` class (implemented in `pid.cpp`). Steering angle using a PID controller is given by :

```
Steer_angle = - (K_p * error_p + K_i * error_i + K_d * error_d)  
```   

Here, K_p, K_i, K_d are the gains for PID components. error_p is the cross-track error (CTE), error_i is the integral of CTE and error_d is the differential of CTE.

There are a few scalings performed beforehand. Firstly, the CTE is divided by a factor of 8. This ensures that steering angle approximately lies between -1 and 1 for `K_p ~ 1` and makes tuning easier later on. Further, `error_i` is multiplied by a factor `dt` and `error_d` is divided by a factor `dt`. I chose`dt = 0.07` which then ensures that `K_i` and `K_d` are also approximately 1 in order to achieve steering angle between -1 and 1.

Now, starting with `K_p = K_i = K_d = 1`, I find that the car in simulation already manages to stay on track! (although oscillating considerably). To understand the effect of each component, first I turn off `K_i` and `K_d`. Here is the video in presence of only P component :

  [![P only](http://img.youtube.com/vi/sfclZrJ0daI/0.jpg)](http://www.youtube.com/watch?v=sfclZrJ0daI)
 
 As seen in the video above, the car starts oscillating wildly before running off the track. This oscillatory behavior is expected with proportional control. Next, I turn on D component. The oscillations go down compared to P-only case and car makes it to  a much farther distance before running off the track. Here is a video demonstarting how far the car makes it with P and D : 
 
 [![P and D](http://img.youtube.com/vi/F6vCwze9AFI/0.jpg)](http://www.youtube.com/watch?v=F6vCwze9AFI)
 
 Finally, turning on all P,I,D components, car is able to complete full lap although still oscillating considerbaly. Final step is tuning the gain parameters while achieving maximum speed possible. I set a constant `throttle = 0.45` and tuned gain parameters using twiddle algorithm which is implemented in the `pid.optimize` function. Because the total error (i.e CTE squared) can vary significantly across different patches of the track, each step in the twiddle algorithm is evaluated across full lap making the entire process time consuming. To solve this problem, `pid.optimize` function performs optimization in real time during simulation improving the gain parameters at each lap. Final gain parameters were chosen to be `K_p = K_i = K_d = 0.35`. The car was able to achieve a maximum speed of **52 mph** using a constant throttle of 0.45 while managing to stay on track (as observed for 20 laps). Here is the full video for one lap :   
  [![PID steering](http://img.youtube.com/vi/SrHDlw_z_qw/0.jpg)](http://www.youtube.com/watch?v=SrHDlw_z_qw)
  
  
# Thottle controller

To achieve maximum possible speed, I have implemented a P-controller for throttle. This part is implemented in `src/main_extra.cpp` (please modify makefile to compile or simply rename this file to `src/main.cpp`). There are a few modifications I have made to deal with high speeds. Firstly, note that steering angle compensation needed at high speeds is smaller compared to that at low speeds, otherwise the car goes out of control. To this end, I have modified CTE for steering angle as follows :

```
CTE_new = CTE/8 * max(0.15, 1 - speed/80) 
```   
With this formula, CTE is much smaller at high speeds. 

Next for throttle controller, I have used the same `pid` class as that for steering but only included P-component. The 'CTE' for throttle is defined as follows :
```
CTE_throttle = (|steer_angle|/5 + |CTE|) * speed/60 
                 + (speed - 100)/25
```   
In the first line, the idea is that throttle error should be proportional to the absolute value of steering angle and absolute value of CTE. The logic for multilplying by speed is as follows. Lets imagine a situation where the car is barely moving and yet it is far away form center of track so that CTE is large. In that case, we do not want throttle to be small even though CTE is large, otherwise car will never move and recover.  The second line has the term `(speed - 100)` which ensures when steering angle and CTE are small, the car will try to speed until it reaches max speed of 100 mph. The coefficients of various terms were manually tuned and gain parameters set to  `K_p = K_i = K_d = 1` for steering and  `K_p = 2.2`,  `K_i = K_d = 0` for throttle, although there is scope for improvement using twiddle algorithm. Here is the final video :    

 [![PID steering](http://img.youtube.com/vi/e2h3u3X-RCc/0.jpg)](http://www.youtube.com/watch?v=e2h3u3X-RCc)
  
  
 Car manages to stay on track (as observed for 20 laps) achieving top speed between **75-85 mph** for each lap.
