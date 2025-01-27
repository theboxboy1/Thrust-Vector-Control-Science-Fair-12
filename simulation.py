from numpy import *
import pylab

delta_time = 0.02   # Update rate of simulation
Inertia = 0.0463   #Inertia about y axis of cylinder is 0.01194 kg*m^2 and of nose cone is simulated on Fusion360 to be 0.0002062 kg*m^2 
num_samples = 200   # Total runtime = delta_time*num_samples
d = 0.307     #Distance between rocket motor and Cg in m
theta0 = 0 #setpoint angle


#--- PID Values ---#

Kp = 50
Ki = 0.05
Kd = 5

#------------------#

theta = zeros(num_samples)  # Creates empty list full of zeros to be populated later
t = [x*delta_time for x in range(num_samples)]  # Creates list of time values whose length is based on number of samples taken(200)
torque = zeros(num_samples)
setpoint = zeros(num_samples)

# Rocket gimbal angles
maxangle = 5.5  
max_rotation_per_step = 200*delta_time  #100mS/60deg per spec sheet, max change in servo angle per step

# E12-4 motor thrust curve from https://www.thrustcurve.org/motors/Estes/E12/
def force(t):

    if t < 0.052:
        return 0
    elif t < 0.096:
        return 5.045
    elif t < 0.196:
        return 9.910
    elif t < 0.251:
        return 24.144
    elif t < 0.287:
        return 31.351
    elif t < 0.300:
        return 32.973
    elif t < 0.344:
        return 29.910
    elif t < 0.370:
        return 17.117
    elif t < 0.400:
        return 14.414
    elif t < 0.500:
        return 12.973
    elif t < 0.600:
        return 11.712
    elif t < 0.700:
        return 11.171
    elif t < 0.800:
        return 10.631
    elif t < 0.900:
        return 10.090
    elif t < 1.000:
        return 9.730
    elif t < 1.101:
        return 9.550
    elif t < 1.200:
        return 9.910
    elif t < 1.300:
        return 9.550
    elif t < 1.400:
        return 9.730
    elif t < 1.500:
        return 9.730
    elif t < 1.600:
        return 9.730
    elif t < 1.700:
        return 9.730
    elif t < 1.800:
        return 9.550
    elif t < 1.900:
        return 9.730
    elif t < 2.000:
        return 9.730
    elif t < 2.100:
        return 9.550
    elif t < 2.200:
        return 9.550
    elif t < 2.300:
        return 9.730
    elif t < 2.375:
        return 9.190
    elif t < 2.400:
        return 9.370
    elif t < 2.440:
        return 5.950
    else:
        return 0

# Initial simulation conditions 
initial_theta = -10
theta[0] = initial_theta*pi/180   # Degrees to radians
theta[1] = initial_theta*pi/180   
running_sum = theta[0] + theta[1]


for n in range(2, num_samples):
    
    #-------------------- PHYSICS ------------=------------#
    
    # Torque is updated based on the rocket thrust curve force
    torque[n-1] = d * force(n*delta_time) * sin(pi * setpoint[n-1]/180)
    
    
    # Torque equation
    theta[n] = 2*theta[n-1] - theta[n-2] + torque[n-2] * (delta_time**2)/Inertia 
              
        
    #-------------------------------------------------------------------------------#
        
        
    
    
    
    
    
    
    #------------------------------------PID Control ----------------------------------------------------#
    
    error = theta[n] - theta0
    P = Kp*error
    
    running_sum = running_sum + (theta[n] - theta0)
    I = Ki*running_sum
    
    D = Kd*(theta[n] - theta[n-1])/delta_time 
    
    
    desired_setpoint = -(P + I + D) # Negative to oppose rocket forces
    
    
    if desired_setpoint - setpoint[n-1] > max_rotation_per_step:
        setpoint[n] = setpoint[n-1] + max_rotation_per_step
        
    elif setpoint[n-1] - desired_setpoint > max_rotation_per_step:
        setpoint[n] = setpoint[n-1] - max_rotation_per_step
        
    else:
        setpoint[n] = desired_setpoint
    
    if setpoint[n] > maxangle:
        setpoint[n] = maxangle
        
    elif setpoint[n] < -maxangle:
        setpoint[n] = -maxangle
    
    #---------------------------------------------------------------------------------------------------------#
    
    print(f"Time Step: {n} theta: {theta[n]} Setpoint: {setpoint[n]}")




# Plotting data, blue is gimbal angle and black line is rocket angle 
p = pylab.plot(t, 180/pi*theta, 'k', label = "Rocket Pitch Angle")
pylab.grid()
pylab.plot(t, setpoint, 'b', label = "Gimbal Angle")    
pylab.xlabel('Time (s)')
pylab.ylabel('Angle (degrees)')
pylab.axhline(y=theta0, color='k', linestyle='--')

pylab.legend()
pylab.plt.show()

#------------------------------ Dynamic equations of motion ----------------------------------------------#


# applied torque = I * alpha = I*angular acceleration = I* d/dt*(d/dt(theta) )=I*d/dt ( theta[n]-theta[n-1] )/delta_time
# applied torque = I/delta_time*() (  theta[n]-theta[n-1] )-(theta[n-1]-theta[n-2] ) )/delta_time
# applied torque = I/delta_time/delta_time*(theta[n]+theta[n-2]-2*theta[n-1])

#----------------------------------------------------------------------------------------------------------#
