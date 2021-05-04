# SAUVC Control

## A. Members
1. Tan Zheng Yang
2. Tran Nguyen Bao Long
3. Chew Lijie Bryan
4. Kushagra Jain
5. Ning Zhi Yan

## B. Summary
### 1. PID Control System
#### a. Useful links 
1. [Matlab Playlist](https://m.youtube.com/watch?v=wkfEZmsQqiA&t=509s)
2. [MIT math intro](https://ocw.mit.edu/courses/mechanical-engineering/2-154-maneuvering-and-control-of-surface-and-underwater-vehicles-13-49-fall-2004/lecture-notes/lec16.pdf)
3. [Seniors' code - Python](https://github.com/heyuhang0/SAUVC2019/blob/111a2ac5936b95c75930394a3df63536a47d61e9/src/auv_qualification_imu.py)
4. [Seniors' code - ROS](https://github.com/heyuhang0/SAUVC2019-ROS)

#### b. Basics
- **Proportional-Integral-Derivative** design: 
    - Terms: 
        - **P**: the proportional part of this control law will create a control action that scales linearly with the error – we often think of this as a spring-like action. 
        - **I**: the integrator is accumulating the error signal over time, and so the control action from this part will continue to grow as long as an error exists. 
        - **D**: the derivative action scales with the derivative of the error. The controller will retard motion toward zero error, which helps to reduce overshoot.)
    - If `u` is the output from the controller, and `e` is the error signal it receives, this control law has the form:<br>
![](https://i.imgur.com/oP2ZBcU.png)
    - Variations: P, PI, PD, PID
    
- [Tuning](http://robotsforroboticists.com/pid-control/): 
    - Terms:
        - **Proportional Term (`Kp`)**: primary term for controlling the error. This directly scales your error: small `Kp` the controller will make small attempts to minimize the error, large `Kp` the controller will make a larger attempt. If the `Kp` is too small you might never minimize the error (unless you are using D and I terms) and not be able to respond to changes affecting your system, and if `Kp` is too large you can have an unstable (ie. weird oscillations) filter that severely overshoot the desired value.
        - **Integral Term (`Ki`)**: lets the controller handle errors that are accumulating over time.The problem is that if you have a large `Ki` you are trying to correct error over time so it can interfere with your response for dealing with current changes. This term is often the cause of instability in your PID controller.
        - **Derivative Term (`Kd`)**: looking at how your system is behaving between time intervals. This helps dampen your system to improve stability.
    - Manual Tuning:  first set `Ki` and `Kd` values to zero. Increase the `Kp` until the output of the loop oscillates (or just performs well), then the `Kp` should be set to approximately half of that value for a “quarter amplitude decay” type response. Then increase `Ki` until any offset is corrected in sufficient time for the process. However, too much `Ki` will cause instability. Finally, increase `Kd`, if required, until the overshooting is minimized. However, too much `Kd` will cause slow responses and sluggishness. A fast PID loop tuning usually overshoots slightly to reach the setpoint more quickly; however, some systems cannot accept overshoot, in which case an over-damped closed-loop system is required, which will require a `Kp` setting significantly less than half that of the KP setting that was causing oscillation. <br>
    ![](https://i.imgur.com/oPkPzJX.png)

    - Ziegler–Nichols: set `Kp` first like above<br>
    ![](https://i.imgur.com/qiPydMp.png)


## C. Tasks
- Input/Output value:
-> Pin and Clarence
- Integrate into ROS nodes to run with hardware ([Ref](https://github.com/heyuhang0/SAUVC2019-ROS/blob/master/bubbles_pid/launch/bubbles_pid.launch))
-> Bryan, ZY and Blong
- Tune
-> Zhi Yan, Kush, ZY and Blong
