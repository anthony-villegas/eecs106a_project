import matplotlib.pyplot as plt
import numpy as np


def FindOmega():
#inputs:
#angle of release - theta
#motor rpm - omega
#arm length - l (pivot at base)
#distance between target & release base (d)

#equation of projectile motion
# x = v0*cos(theta)*t
# y = v0*sin(theta)*t-1/2*g*t^2
# v0 = omega/l
# delta_theta = omega*t_r (t_r = time of release)
# motor rise time function of rpm (0 - target rpm)
# t_r_error release time error (in ms)

    g = 9.81
    l = float(input("\nInput arm length: ")) #0.1778m default (7" arm length & 6" from center to ball)

    if l == 0:
        l = 0.1778
    
    #
    d = float(input("\nSpecify the distance from target: "))
    
    
    #uncomment this line below and comment the line above if you want to use time_release
    #t_r = float(input("\nProvide time of release in ms: ")) #assumed to be in ms after passing the horizontal

    t_r_error = float(input("\nProvide expected time release error in ms: "))/1000

    theta = 45*np.pi/180 #45 strict deg

    #d_sender = np.cos(theta)*(l-0.0254) - 1" away from tip of arm
    d_receiver = np.cos(theta)*(l-0.0254) #horizontal distance from receiver center to catcher center @45 deg
    d_full = d+2*d_receiver
    v0 = np.sqrt(g*d_full/np.sin(2*theta))
    print('\nInitial Velocity at release', v0)

    omega = v0/l
    t_r = theta/omega

    theta_band = np.array([[omega*(t_r-t_r_error)],
                            [theta],
                            [omega*(t_r+t_r_error)]])


    print('Required motor rpm (loaded): ',omega*60/(2*np.pi))
    flight_time = []
    fd = []
    vd = []
    vf = []
    theta_offset = []

    for i in range(len(theta_band)):

        x = []
        y = []



        flight_time.append(2*v0*np.sin(theta_band[i][0])/g)
        fd.append(0.5*1.225*0.5*v0**2*np.pi/4*0.0254**2/0.0115)
        vd.append(fd[i] * flight_time[i])
        vf.append(v0-vd[i])
        theta_offset.append(((np.pi-np.pi/4) - omega*flight_time[i])*180/np.pi)

        while abs(theta_offset[i]) > 360:
            theta_offset[i] += 360


        if i == 0:
            print('\n Flight time is: ', flight_time[i],' seconds if released at time: ', t_r-t_r_error, 'seconds,', t_r_error,' seconds too early')
            print('Final Velocity when accounting for drag: ',vf[i])
            print('Offset Theta of receiving arm needs to be at:',theta_offset[i],' at time of release.')
            
        
        if i == 1:
            print('\n Flight time is: ', flight_time[i],' seconds if released at time: ', t_r, ' seconds')
            print('Release signal should be given at theta = ',theta_band[0][0]*180/np.pi,' to release at 45 deg accounting for delay')
            print('Final Velocity when accounting for drag: ',vf[i])
            print('Offset Theta of receiving arm needs to be at:',theta_offset[i],' at time of release.')

        if i == 2:
            print('\n Flight time is: ', flight_time[i],' seconds if released at time: ', t_r+t_r_error, 'seconds,', t_r_error,' seconds too late')
            print('Final Velocity when accounting for drag: ',vf[i])
            print('Offset Theta of receiving arm needs to be at:',theta_offset[i],' at time of release.')

        time_step = flight_time[i]/50
        time = 0
        
        while time < flight_time[i]+flight_time[i]/50:
            xt = v0*np.cos(theta_band[i][0])*time
            yt = v0*np.sin(theta_band[i][0])*time - 0.5*g*time**2
        
            x.append(xt)
            y.append(yt)
            time += time_step

        print('Distance covered: ',x[-1],' meters within: ',(d_full-x[-1]),' m of target')


        plt.plot(x,y, label=r'Trajectory released at $\theta$ = ' + "{:.2f}".format(90-(theta_band[i][0]*180/np.pi)) + ' deg')
        plt.legend()

    
    
    plt.title('Projectile path with 30 ms variance in release')
    plt.xlabel('Distance (m)')
    plt.ylabel('Height (m)')
    plt.show()
    

FindOmega()

