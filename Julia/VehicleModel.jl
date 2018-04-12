module VehicleModel

lf = 0.9    # distance tire to COG
lr = 0.640
lb = 1.440    #width of car
beta    = 0.0    #slip angle
gammaf  = 0.0  #steering angle
A       = 2.5    #vehicle cross section
FEngine = 4000   # 5000N for car
mass    = 170   #kg
Cd      = 0.5   #drag coefficient
p       = 1.225  # air desity in kg/m^3
A       = 2.0    #vehicle cross section
Crr     = 0.014  #roll resistance coefficient
rad     = 0.2    #radius of tires in m
max_speed = 43/3.6 # 120km/h /3.6 = m/s
max_long_acc = 10   #m/s**2 longitudinal acceleration max
max_long_dec = 10   #m/s**2 longitudinal deceleration max
max_lat_acc = 20  # 2g lateral acceleration
max_steering_angle = (30.0/180.0)*pi #
max_acceleration_time = 4.0 #seconds


struct CarPose
    x::Float64
    y::Float64
    x_d::Float64
    psi::Float64
    y_d::Float64
    psi_d::Float64
end

struct CarControls
    throttle::Float64
    steer::Float64
end

struct CarState
    x::Float64
    y::Float64
    x_d::Float64
    psi::Float64
    y_d::Float64
    psi_d::Float64
    throttle::Float64
    steer::Float64

end
# get last state and calculate next state for it + shift the whole vector to create the new stateVector
function createNewStateVector(sV, dt, N) # sV für stateVector
    carPose = CarPose(sV[end-7], sV[end-6], sV[end-5], sV[end-4], sV[end-3], sV[end-2])
    carControl = CarControls(sV[end-1], sV[end])
    carPose = computeTimeStep(carPose, carControl, dt)
    sV = circshift(sV, -8)
    sV[end-2] = carPose.psi_d
    sV[end-3] = carPose.y_d
    sV[end-4] = carPose.psi
    sV[end-5] = carPose.x_d
    sV[end-6] = carPose.y
    sV[end-7] = carPose.x
    return sV
end

function computeTimeStep(carPose, carControl, dt)
    beta = atan((lr/(lf + lr)) * tan(carControl.steer))
    max_beta =  atan(1.0/2 * (lf + lr) * max_lat_acc / carPose.x_d^2)
    if (beta >= 0)
        beta = min(beta, max_beta)
    else
        beta = - min(-beta, max_beta)
    end
    x = carPose.x + carPose.x_d * dt * cos(carPose.psi + beta)
    y = carPose.y + carPose.x_d * dt * sin(carPose.psi + beta)
    psi = carPose.psi + (carPose.x_d * dt/lr) * sin(beta)
    x_d = carPose.x_d + carControl.throttle * dt
    x_d = (x_d > max_speed) ?  max_speed : x_d
    y_d = 0
    psi_d = 0
    carPose = CarPose(x, y, x_d, psi, y_d, psi_d)
    return carPose
end


export CarState, computeTimeStep, CarPose, CarControls, max_steering_angle, max_long_acc, max_long_dec, createNewStateVector

end

#=
using VehicleModel

carState = VehicleModel.CarState(0,0,0.1,pi,0,0,0.1)
carPose = VehicleModel.CarPose(0,0,0,0)
carControl = VehicleModel.CarControls(1, 0)
carPose = VehicleModel.computeTimeStep(carPose, carControl, 0.1)
pose = VehicleModel.computeTimeStep(carPose, carControl, 0.1)
println(pose)
=#
