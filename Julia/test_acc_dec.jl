using PyPlot
include("RaceCourse.jl")
include("VehicleModel.jl")
include("MPC.jl")
include("circular_buffer.jl")
include("smallMPC.jl")
#using CircBuffer
#using VehicleModel.CarPose, VehicleModel.CarState, VehicleModel
#using RaceCourse









#create manual input
#res vector for maximal acceleration
n1, n2, n3 = 150, 65, 0
tmp =  [10,0]
tmp2 = [-10,0]
tmp3 = [0,-VehicleModel.max_steering_angle]
controlVectorThrottle = []
controlVectorSteer = []
for i in 1:n1
    controlVectorThrottle = vcat(controlVectorThrottle, tmp[1])
    controlVectorSteer = vcat(controlVectorSteer, tmp[2])

end
for i in 1:n2
    controlVectorThrottle = vcat(controlVectorThrottle, tmp2[1])
    controlVectorSteer = vcat(controlVectorSteer, tmp2[2])
end
for i in 1:n3
    controlVectorThrottle = vcat(controlVectorThrottle, tmp3[1])
    controlVectorSteer = vcat(controlVectorSteer, tmp3[2])
end

max_steps = n1 + n2 + n3


dt = 0.05


stateVectorKin= VehicleModel.CarPose(0,0,0.1,0, 0, 0)
stateVectorNonLinear_Base = VehicleModel.CarPose(0,0,0.1,0, 0, 0)
stateVectorNonLinear_Enhanced_Lat = VehicleModel.CarPose(0,0,0.1,0, 0, 0)
stateVectorNonLinear_Enhanced_Lat_Cmplx = VehicleModel.CarPose(0,0,0.1,0, 0, 0)
stateVectorNonLinear_Enhanced_Long_Cmplx = VehicleModel.CarPose(0,0,0.1,0, 0, 0)


kin_psi = []
dyn_base_psi = []
dyn_lat_psi = []
dyn_lat_cplx_psi = []
dyn_long_cplx_speed = []
max_speed = []

for i in 1:max_steps
    carControl = VehicleModel.CarControls(controlVectorThrottle[i], controlVectorSteer[i])

    stateVectorKin =  VehicleModel.kin_bycicle_model(stateVectorKin, carControl, dt)
    max_speed = vcat(max_speed, stateVectorKin.x_d)

    stateVectorNonLinear_Enhanced_Long_Cmplx =  VehicleModel.dyn_model_enhanced_long_cmplx(stateVectorNonLinear_Enhanced_Long_Cmplx, carControl, dt)
    dyn_long_cplx_speed = vcat(dyn_long_cplx_speed, stateVectorNonLinear_Enhanced_Long_Cmplx.x_d)

end


print("size of kin_psi", length(kin_psi))
print("max_speed", max_speed[indmax(max_speed)])

lin = linspace(1, max_steps, max_steps)

areas = 5*ones(lin)
subplot(211)
scatter(lin,controlVectorThrottle,s=areas,alpha=1.0)
grid()
xlabel("Time Steps")
ylabel("Throttle Input")
title("Driver Input")
ax = gca()
subplot(212)
xlabel("Time Steps")
ylabel("Speed in m/s")
title("Orientation of Car")
scatter(lin,dyn_long_cplx_speed,s=areas,alpha=1.0)
scatter(lin,max_speed,s=areas,alpha=1.0)

grid()
