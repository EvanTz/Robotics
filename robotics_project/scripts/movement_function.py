# Tzortzis Evangelos 3088
# Mpakolas Theodoros 3035
# Zamparas Nikolaos  2969

import math
import matplotlib.pyplot as plt

# Linear Segments with Parabolic Blends (LSBP) functions

#maximum velocities
max_angular_velocity = 0.6981
max_linear_velocity = 0.2

# time segments calculation
def calc_time_segments(x_initial,y_initial,theta_initial,x_inter,y_inter,theta_inter,x_final,y_final,theta_final):

    t_0 = 0

    t_1 = round(abs(theta_inter-theta_initial)*10/(9*max_angular_velocity),2)

    t_2 = round(math.sqrt((abs(x_inter-x_initial)*10)**2 + (abs(y_inter-y_initial)*10)**2)/(9*max_linear_velocity),2)

    t_3 = round(abs(math.atan2(y_final-y_inter,x_final-x_inter) - theta_inter)*10/(9*max_angular_velocity),2)

    t_4 = round(math.sqrt((abs(x_final-x_inter)*10)**2 + (abs(y_final-y_inter)*10)**2)/(9*max_linear_velocity),2)

    t_5 = round(abs(theta_final-math.atan2(y_final-y_inter,x_final-x_inter))*10/(9*max_angular_velocity),2)

    # print('t_0:',t_0)
    # print('t_1:',t_1)
    # print('t_2:',t_2)
    # print('t_3:',t_3)
    # print('t_4:',t_4)
    # print('t_5:',t_5)

    return t_0,t_1,t_2,t_3,t_4,t_5

# first parabolic segment of the parabolic blends
def calc_first_part(current_time, p_initial, p_final, acceleration , start_time):
    # position the current time
    p_ct = p_initial + (1/2) * acceleration * (current_time - start_time)**2 * math.copysign(1,(p_final-p_initial))
    # speed the current time
    p_dot_ct = acceleration * (current_time - start_time)
    # acceleration the current time
    p_dot_dot_ct = acceleration

    return p_ct, p_dot_ct , p_dot_dot_ct

# linear segment between parabolic segments
def calc_middle_part(current_time, start_time, p_initial, p_final, acceleration , t_b):
    # position the current time
    x_t_b,_,_ = calc_first_part(start_time+t_b, p_initial=p_initial, p_final= p_final, acceleration = acceleration , start_time = start_time)
    p_ct = x_t_b + acceleration * t_b * (current_time - start_time - t_b) * math.copysign(1,(p_final-p_initial))
    # speed the current time
    p_dot_ct = acceleration * t_b
    # acceleration the current time
    p_dot_dot_ct = 0

    return p_ct, p_dot_ct , p_dot_dot_ct

# last parabolic segment of the parabolic blends
def calc_last_part(current_time, p_initial, p_final, acceleration , start_time, finish_time):
    # position the current time
    p_ct = p_final - 0.5 * acceleration * (start_time + finish_time - current_time)**2 * math.copysign(1,(p_final-p_initial))
    # speed the current time
    p_dot_ct = acceleration * (start_time + finish_time - current_time)
    # acceleration the current time
    p_dot_dot_ct = - acceleration

    return p_ct, p_dot_ct , p_dot_dot_ct


# calculate the trajectory based on current_time
def trajectory_calc(current_time,
                    t_0=0,t_1=0.78,t_2=94.5,t_3=3.28,t_4=89,t_5=5,
                    x_0=0,y_0=0,theta_0=0,
                    x_1=15,y_1=8,theta_1=0.4899, 
                    x_f=15,y_f=-8,theta_final=1.544):
    
    # Cases:

    # 1. the robot turns from theta_0 to theta_1

    # 2. the robot moves from x_0,y_0 to x_1,y_1 in a straight line

    # 3. the robot turns from theta_1 to theta_2
    theta_2 = math.atan2(y_f-y_1,x_f-x_1) # = -π/2 gia dexiostrofh kinhsh

    # 4. the robot moves from x_1,y_1 to x_f,y_f in a straight line

    # 5. the robot turns to final orientation from theta_2 to theta_final
 
    if current_time <= t_1:
        # case 1
        t_b = 0.1 * t_1 # t_b is 10% of movement time 
        acc = abs(theta_1-theta_0) * 100 / (9 * t_1**2)

        # first parabolic blend
        if current_time <= t_b:
            theta,theta_speed,theta_acc = calc_first_part(current_time=current_time, start_time=t_0, p_initial=theta_0, p_final=theta_1, acceleration=acc)

        # middle linear segment
        elif current_time <= t_1-t_b:
            theta,theta_speed,theta_acc = calc_middle_part(current_time=current_time, start_time=t_0, p_initial=theta_0, p_final=theta_1, acceleration=acc, t_b=t_b)

        # last parabolic blend
        else:
            theta,theta_speed,theta_acc = calc_last_part(current_time=current_time, p_initial=theta_0, p_final=theta_1, acceleration=acc, start_time=t_0, finish_time= t_1)

        x = x_0
        x_speed = 0
        x_acc = 0

        y = y_0
        y_speed = 0
        y_acc = 0  
        
    elif current_time <= t_1+t_2:
        # case 2
        t_b = 0.1 * t_2 # t_b is 10% of movement time 
        acc_x = abs(x_1-x_0) * 100 / (9 * t_2**2)
        acc_y = abs(y_1-y_0) * 100 / (9 * t_2**2)

        # first parabolic blend
        if current_time <= t_1+t_b:
            x,x_speed,x_acc = calc_first_part(current_time=current_time, start_time=t_1, p_initial=x_0, p_final=x_1, acceleration=acc_x)
            y,y_speed,y_acc = calc_first_part(current_time=current_time, start_time=t_1, p_initial=y_0, p_final=y_1, acceleration=acc_y)

        # middle linear segment
        elif current_time <= t_1+t_2-t_b:
            x,x_speed,x_acc = calc_middle_part(current_time=current_time, start_time=t_1, p_initial=x_0, p_final=x_1, acceleration=acc_x, t_b=t_b)
            y,y_speed,y_acc = calc_middle_part(current_time=current_time, start_time=t_1, p_initial=y_0, p_final=y_1, acceleration=acc_y, t_b=t_b)


        # last parabolic blend
        else:
            x,x_speed,x_acc = calc_last_part(current_time=current_time, p_initial=x_0, p_final=x_1, acceleration=acc_x, start_time=t_1, finish_time= t_2)
            y,y_speed,y_acc = calc_last_part(current_time=current_time, p_initial=y_0, p_final=y_1, acceleration=acc_y, start_time=t_1, finish_time= t_2)
        
        theta = theta_1
        theta_speed = 0
        theta_acc = 0

    elif current_time <= t_1+t_2+t_3:
        # case 3
        t_b = 0.1 * t_3 # t_b is 10% of movement time 
        acc = abs(theta_2-theta_1) * 100 / (9 * t_3**2)

        # first parabolic blend
        if current_time <= t_1+t_2+t_b:
            theta,theta_speed,theta_acc = calc_first_part(current_time=current_time, start_time=t_1+t_2, p_initial=theta_1, p_final=theta_2, acceleration=acc)

        # middle linear segment
        elif current_time <= t_1+t_2+t_3-t_b:
            theta,theta_speed,theta_acc = calc_middle_part(current_time=current_time, start_time=t_1+t_2, p_initial=theta_1, p_final=theta_2, acceleration=acc, t_b=t_b)

        # last parabolic blend
        else:
            theta,theta_speed,theta_acc = calc_last_part(current_time=current_time, p_initial=theta_1, p_final=theta_2, acceleration=acc, start_time=t_1+t_2, finish_time= t_3)

        theta_speed =-theta_speed # negative speed to move clockwise

        x = x_1
        x_speed = 0
        x_acc = 0

        y = y_1
        y_speed = 0
        y_acc = 0  

    elif current_time <= t_1+t_2+t_3+t_4:
        # case 4
        t_b = 0.1 * t_4 # t_b is 10% of movement time 
        acc_x = abs(x_f-x_1) * 100 / (9 * t_4**2)
        acc_y = abs(y_f-y_1) * 100 / (9 * t_4**2)

        # first parabolic blend
        if current_time <= t_1+t_2+t_3+t_b:
            x,x_speed,x_acc = calc_first_part(current_time=current_time, start_time=t_1+t_2+t_3, p_initial=x_1, p_final=x_f, acceleration=acc_x)
            y,y_speed,y_acc = calc_first_part(current_time=current_time, start_time=t_1+t_2+t_3, p_initial=y_1, p_final=y_f, acceleration=acc_y)

        # middle linear segment
        elif current_time <= t_1+t_2+t_3+t_4-t_b:
            x,x_speed,x_acc = calc_middle_part(current_time=current_time, start_time=t_1+t_2+t_3, p_initial=x_1, p_final=x_f, acceleration=acc_x, t_b=t_b)
            y,y_speed,y_acc = calc_middle_part(current_time=current_time, start_time=t_1+t_2+t_3, p_initial=y_1, p_final=y_f, acceleration=acc_y, t_b=t_b)


        # last parabolic blend
        else:
            x,x_speed,x_acc = calc_last_part(current_time=current_time, p_initial=x_1, p_final=x_f, acceleration=acc_x, start_time=t_1+t_2+t_3, finish_time= t_4)
            y,y_speed,y_acc = calc_last_part(current_time=current_time, p_initial=y_1, p_final=y_f, acceleration=acc_y, start_time=t_1+t_2+t_3, finish_time= t_4)
        
        theta = theta_2
        theta_speed = 0
        theta_acc = 0 

    elif current_time <= t_1+t_2+t_3+t_4+t_5:
        # case 5
        t_b = 0.1 * t_5 # t_b is 10% of movement time 
        acc = abs(theta_final-theta_2) * 100 / (9 * t_5**2)

        # first parabolic blend
        if current_time <= t_1+t_2+t_3+t_4+t_b:
            theta,theta_speed,theta_acc = calc_first_part(current_time=current_time, start_time=t_1+t_2+t_3+t_4, p_initial=theta_2, p_final=theta_final, acceleration=acc)

        # middle linear segment
        elif current_time <= t_1+t_2+t_3+t_4+t_5-t_b:
            theta,theta_speed,theta_acc = calc_middle_part(current_time=current_time, start_time=t_1+t_2+t_3+t_4, p_initial=theta_2, p_final=theta_final, acceleration=acc, t_b=t_b)

        # last parabolic blend
        else:
            theta,theta_speed,theta_acc = calc_last_part(current_time=current_time, p_initial=theta_2, p_final=theta_final, acceleration=acc, start_time=t_1+t_2+t_3+t_4, finish_time= t_5)

        x = x_f
        x_speed = 0
        x_acc = 0

        y = y_f
        y_speed = 0
        y_acc = 0 

    else:
        x = x_f
        x_speed = 0
        x_acc = 0

        y = y_f
        y_speed = 0
        y_acc = 0 

        theta = theta_final
        theta_speed = 0
        theta_acc = 0   

    ret_list = [x, y, theta, x_speed, y_speed, theta_speed, x_acc, y_acc, theta_acc]
    return ret_list






#############################################| PLOTING OF THE TRAJECTORY |#################################################
if __name__ == '__main__':

    ###### SET DIFFERENT COORDINATES AND ANGLES HERE FOR TESTING ######

    # initial point
    x_0 = 0
    y_0 = 0
    theta_0 = 0

    # intermediate point
    # round(3088/200) = 15 
    # round(3088/400) = 8
    x_i = 15
    y_i = 8

    # final point
    # round(3088/200) = 15 
    # -round(3088/400) = -8
    x_f = 15
    y_f = -8

    # 3088/2000 = 1.544 radians
    theta_f = 1.544

    ###################################################################


    theta_i= math.atan2(y_i-y_0,x_i-x_0)
    
    theta_temp = math.atan2(y_f-y_i,x_f-x_i)

    # time segment durations
    t_0,t_1,t_2,t_3,t_4,t_5 = calc_time_segments(x_0,y_0,theta_0,
                                                x_i,y_i,theta_i,
                                                x_f,y_f,theta_f)


    # time durations (in seconds) for each case 
    # t_0 = 0
    # t_1 = 0.78
    # t_2 = 94.5
    # t_3 = 3.28
    # t_4 = 89
    # t_5 = 5

    t_total = t_0 + t_1 + t_2 + t_3 + t_4 + t_5  # total time for all parts of the movement
    print('Total time: ',t_total)

    steps_per_second = 100 # steps per second / MONO GIA TIS DOKIMES , STO ROS THA ORIZETAI
    steps = int(steps_per_second*t_total) 
    points = [(x/steps)*t_total for x in range(0,steps)]

    result_test = [trajectory_calc(ct,
                    t_0=t_0,t_1=t_1,t_2=t_2,t_3=t_3,t_4=t_4,t_5=t_5,
                    x_0=x_0,y_0=y_0,theta_0=theta_0,
                    x_1=x_i,y_1=y_i,theta_1=theta_i, 
                    x_f=x_f,y_f=y_f,theta_final=theta_f) for ct in points]


    ###################### | PLOTTING | ######################

    # position
    x_test_pos = [v[0] for v in result_test]
    y_test_pos = [v[1] for v in result_test]
    theta_test_pos = [v[2] for v in result_test]

    # # plt.figure(figsize=(50, 10), dpi=200) # VERY HIGH RESOLUTION !!!!
    # plt.figure(figsize=(20, 5), dpi=80)

    # plt.plot(points,theta_test_pos)
    # plt.plot(points,x_test_pos)
    # plt.plot(points,y_test_pos)

    # plt.axvline(t_0,color='red', linestyle='--', linewidth = 0.8)
    # plt.axvline(t_1,color='red', linestyle='--', linewidth = 0.8)
    # plt.axvline(t_1+t_2,color='red', linestyle='--', linewidth = 0.8)
    # plt.axvline(t_1+t_2+t_3,color='red', linestyle='--', linewidth = 0.8)
    # plt.axvline(t_1+t_2+t_3+t_4,color='red', linestyle='--', linewidth = 0.8)
    # plt.axvline(t_1+t_2+t_3+t_4+t_5,color='red', linestyle='--', linewidth = 0.8)

    # plt.xlabel('Time')
    # plt.ylabel('Angle, Coords')
    # plt.legend(['Angle', 'X Coordinate', 'Y Coordinate', 'Time segments'], handlelength=4)
    # plt.grid()
    # plt.title('Trajectory')

    # #plt.show()

    # speed
    x_test_vel = [v[3] for v in result_test]
    y_test_vel = [v[4] for v in result_test]
    theta_test_vel = [v[5] for v in result_test]

    # # plt.figure(figsize=(50, 10), dpi=200) # VERY HIGH RESOLUTION !!!!
    # plt.figure(figsize=(20, 5), dpi=80)

    # plt.plot(points,theta_test_vel)
    # plt.plot(points,x_test_vel)
    # plt.plot(points,y_test_vel)

    # plt.axvline(t_0,color='red', linestyle='--', linewidth = 0.8)
    # plt.axvline(t_1,color='red', linestyle='--', linewidth = 0.8)
    # plt.axvline(t_1+t_2,color='red', linestyle='--', linewidth = 0.8)
    # plt.axvline(t_1+t_2+t_3,color='red', linestyle='--', linewidth = 0.8)
    # plt.axvline(t_1+t_2+t_3+t_4,color='red', linestyle='--', linewidth = 0.8)
    # plt.axvline(t_1+t_2+t_3+t_4+t_5,color='red', linestyle='--', linewidth = 0.8)

    # plt.xlabel('Time')
    # plt.ylabel('Angle, Coords')
    # plt.legend(['Angle', 'X Coordinate', 'Y Coordinate', 'Time segments'], handlelength=4)
    # plt.grid()
    # plt.title('Speed')

    # #plt.show()

    # accelaration
    x_test_acc = [v[6] for v in result_test]
    y_test_acc = [v[7] for v in result_test]
    theta_test_acc = [v[8] for v in result_test]

    # # plt.figure(figsize=(50, 10), dpi=200) # VERY HIGH RESOLUTION !!!!
    # plt.figure(figsize=(20, 5), dpi=80)

    # plt.plot(points,theta_test_acc)
    # plt.plot(points,x_test_acc)
    # plt.plot(points,y_test_acc)

    # plt.axvline(t_0,color='red', linestyle='--', linewidth = 0.8)
    # plt.axvline(t_1,color='red', linestyle='--', linewidth = 0.8)
    # plt.axvline(t_1+t_2,color='red', linestyle='--', linewidth = 0.8)
    # plt.axvline(t_1+t_2+t_3,color='red', linestyle='--', linewidth = 0.8)
    # plt.axvline(t_1+t_2+t_3+t_4,color='red', linestyle='--', linewidth = 0.8)
    # plt.axvline(t_1+t_2+t_3+t_4+t_5,color='red', linestyle='--', linewidth = 0.8)

    # plt.xlabel('Time')
    # plt.ylabel('Angle, Coords')
    # plt.legend(['Angle', 'X Coordinate', 'Y Coordinate', 'Time segments'], handlelength=4)
    # plt.grid()
    # plt.title('Acceleration')

    # # plt.show()

    # plot the coordinates functions
    # plt.style.use('dark_background')
    fig, ax = plt.subplots(3)

    ax[0].set_title('Position')
    ax[1].set_title('Combined Velocity AND x,y velocity components')
    ax[2].set_title('Acceleration')

    # l1, = ax[0].plot(points,theta_test_pos)
    l2, = ax[0].plot(points,x_test_pos)
    l3, = ax[0].plot(points,y_test_pos)    

    # ax[1].plot(points,theta_test_vel)
    ax[1].plot(points,x_test_vel)
    ax[1].plot(points,y_test_vel)

    # combined velocity
    cv = []
    for v in range(len(y_test_vel)):
        cv.append(math.sqrt(x_test_vel[v]**2 + y_test_vel[v]**2))

    ax[1].plot(points,cv,c='r')
    
    # ax[2].plot(points,theta_test_acc)
    ax[2].plot(points,x_test_acc)
    ax[2].plot(points,y_test_acc)

    ts = ax[0].axvline(t_0,color='green', linestyle='--', linewidth = 0.8)

    for i in ax:
        i.axvline(t_0,color='green', linestyle='--', linewidth = 0.9)
        i.axvline(t_1,color='green', linestyle='--', linewidth = 0.9)
        i.axvline(t_1+t_2,color='green', linestyle='--', linewidth = 0.9)
        i.axvline(t_1+t_2+t_3,color='green', linestyle='--', linewidth = 0.9)
        i.axvline(t_1+t_2+t_3+t_4,color='green', linestyle='--', linewidth = 0.9)
        i.axvline(t_1+t_2+t_3+t_4+t_5,color='green', linestyle='--', linewidth = 0.9)

        i.set_xticks([t_0,t_1,t_2+t_1,t_3+t_2+t_1,t_4+t_3+t_2+t_1,t_5+t_4+t_3+t_2+t_1])
        i.tick_params(axis='x', labelsize=7)
        for t in i.get_xticklabels():
            t.set_rotation(90)
        i.set_xlabel('Time')
        i.set_ylabel('Coords')
        # i.set_ylabel('Angle, Coords')
        i.grid()

    ax[0].axhline(x_i,color='blue', linestyle='--', linewidth = 0.8)
    ax[0].axhline(x_f,color='blue', linestyle='--', linewidth = 0.8)
    ax[0].axhline(y_i,color='blue', linestyle='--', linewidth = 0.8)
    ax[0].axhline(y_f,color='blue', linestyle='--', linewidth = 0.8)
    ax[0].set_yticks([x_0,y_0,x_i,x_f,y_i,y_f])

    ax[1].axhline(max_linear_velocity,color='blue', linestyle='--', linewidth = 0.8)
    ax[1].set_yticks([max_linear_velocity,0])

    # fig.legend((l1,l2,l3,ts),('Angle', 'X Coordinate', 'Y Coordinate','Time Segment'),handlelength=4)
    fig.legend((l2,l3,ts),('X Coordinate', 'Y Coordinate','Time Segment'),handlelength=4)

    plt.tight_layout()


    # plot the angle functions
    fig2, ax2 = plt.subplots(3)

    ax2[0].set_title('Position')
    ax2[1].set_title('Velocity')
    ax2[2].set_title('Acceleration')

    l12, = ax2[0].plot(points,theta_test_pos)  

    ax2[1].plot(points,theta_test_vel)

    ax2[2].plot(points,theta_test_acc)

    ts2 = ax2[0].axvline(t_0,color='red', linestyle='--', linewidth = 0.8)

    for i in ax2:
        i.axvline(t_0,color='red', linestyle='--', linewidth = 0.8)
        i.axvline(t_1,color='red', linestyle='--', linewidth = 0.8)
        i.axvline(t_1+t_2,color='red', linestyle='--', linewidth = 0.8)
        i.axvline(t_1+t_2+t_3,color='red', linestyle='--', linewidth = 0.8)
        i.axvline(t_1+t_2+t_3+t_4,color='red', linestyle='--', linewidth = 0.8)
        i.axvline(t_1+t_2+t_3+t_4+t_5,color='red', linestyle='--', linewidth = 0.8)

        i.set_xticks([t_0,t_1,t_2+t_1,t_3+t_2+t_1,t_4+t_3+t_2+t_1,t_5+t_4+t_3+t_2+t_1])
        i.tick_params(axis='x', labelsize=7)
        for t in i.get_xticklabels():
            t.set_rotation(90)
        i.set_xlabel('Time')
        i.set_ylabel('Angle')
        i.grid()

    ax2[0].axhline(theta_i,color='blue', linestyle='--', linewidth = 0.8)
    ax2[0].axhline(theta_temp,color='blue', linestyle='--', linewidth = 0.8)
    ax2[0].axhline(theta_f,color='blue', linestyle='--', linewidth = 0.8)
    ax2[0].set_yticks([theta_0,theta_i,theta_temp,theta_f])

    ax2[1].axhline(max_angular_velocity,color='blue', linestyle='--', linewidth = 0.8)
    ax2[1].set_yticks([max_angular_velocity,0])
    
    fig2.legend((l12,ts2),('Theta Angle','Time Segment'),handlelength=2)

    plt.tight_layout()

    plt.show()
