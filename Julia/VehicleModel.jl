module VehicleModel

lf = 0.9    # distance tire to COG
lr = 0.640
lb = 1.440    #width of car
beta = 0.0    #slip angle
gammaf = 0.0  #steering angle
A     = 2.5    #vehicle cross section
FEngine = 4000   # 5000N for car
mass    = 170   #kg
Cd      = 0.5   #drag coefficient
p       = 1.225  # air desity in kg/m^3
A       = 2.0    #vehicle cross section
Crr     = 0.014  #roll resistance coefficient
max_speed = 60/3.6 # 120km/h /3.6 = m/s
max_long_acc = 10   #m/s**2 longitudinal acceleration max
max_long_dec = 10   #m/s**2 longitudinal deceleration max
max_lat_acc = 20  # 2g lateral acceleration
max_steering_angle = (30.0/180.0)*pi #
max_acceleration_time = 4.0 #seconds

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
