module VehicleModel

#vehicle geometry
lf  = 1.09    # distance tire to COG
lr  = 0.9
lb  = 1.99    #width of car
Af  = 2.25
rad = 0.2    #radius of tires in m
mass= 600   #kg


#values for longitudinal computation
P_Engine= 50000  #Watt
Cd      = 1.083   #drag coefficient
rho     = 1.225  # air desity in kg/m^3
Crr     = 0.014  #roll resistance coefficient
mu	    = 0.0027   #roll resistance

#values to limit car_parameters for mpc
max_speed = 43/3.6 # 120km/h /3.6 = m/s
max_long_acc = 10   #m/s**2 longitudinal acceleration max
max_long_dec = 10   #m/s**2 longitudinal deceleration max
max_lat_acc = 20  # 2g lateral acceleration
max_steering_angle = (30.0/180.0)*pi #


#tire model
Df   = 2984.0 #N
Db   = 3274.0 #N
xmf  = 0.25 #rad
xmb  = 0.37 #rad
betaf = pi/2.0 - 0.00001#rad
betab = pi/2.0 - 0.00001#rad
yaf  = 2952.0 #N
yab  = 3270.0 #N #ya has to be smaller than D


struct CarPose
    x::Float64
    y::Float64
    v::Float64
    yaw::Float64
end
struct CarControls
    acc::Float64
    steer::Float64
end
struct CarState
    x::Float64
    y::Float64
    v::Float64
    yaw::Float64
    acc::Float64
    steer::Float64

end
# get last state and calculate next state for it + shift the whole vector to create the new stateVector
function createNewStateVector(sV, dt, N) # sV für stateVector
    carPose = CarPose(sV[end-5], sV[end-4], sV[end-3], sV[end-2])
    carControl = CarControls(sV[end-1], sV[end])
    carPose = computeTimeStep(carPose, carControl, dt)
    sV = circshift(sV, -6)
    sV[end-2] = carPose.yaw
    sV[end-3] = carPose.v
    sV[end-4] = carPose.y
    sV[end-5] = carPose.x
    return sV
end

function computeTimeStep(carPose, carControl, dt)
    beta = atan((lr/(lf + lr)) * tan(carControl.steer))
    max_beta =  atan(1.0/2 * (lf + lr) * max_lat_acc / carPose.v^2)
    if (beta >= 0)
        beta = min(beta, max_beta)
    else
        beta = - min(-beta, max_beta)
    end
    x_1 = carPose.x + carPose.v * dt * cos(carPose.yaw + beta)
    y_1 = carPose.y + carPose.v * dt * sin(carPose.yaw + beta)
    yaw_1 = carPose.yaw + (carPose.v * dt/lr) * sin(beta)
    v_1 = carPose.v + carControl.acc * dt
    v_1 = (v_1 > max_speed) ?  max_speed : v_1
    carPose = CarPose(x_1, y_1, v_1, yaw_1)
    return carPose
end

function vehicleModel_1(carPose, carControl, dt):
end


export CarState, computeTimeStep, CarPose, CarControls, max_steering_angle, max_long_acc, max_long_dec, createNewStateVector

end
