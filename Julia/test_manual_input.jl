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
n1, n2, n3 = 5, 30, 30
tmp =  [10,0]
tmp2 = [0,VehicleModel.max_steering_angle]
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


stateVectorKin= VehicleModel.CarPose(0,0,20,0, 0, 0)
stateVectorNonLinear_Base = VehicleModel.CarPose(0,0,20,0, 0, 0)
stateVectorNonLinear_Enhanced_Lat = VehicleModel.CarPose(0,0,20,0, 0, 0)
stateVectorNonLinear_Enhanced_Lat_Cmplx = VehicleModel.CarPose(0,0,20,0, 0, 0)


kin_psi = []
dyn_base_psi = []
dyn_lat_psi = []
dyn_lat_cplx_psi = []
max_speed = []

for i in 1:max_steps
    carControl = VehicleModel.CarControls(controlVectorThrottle[i], controlVectorSteer[i])

    stateVectorKin =  VehicleModel.kin_bycicle_model(stateVectorKin, carControl, dt)
    kin_psi = vcat(kin_psi, stateVectorKin.psi)
    max_speed = vcat(max_speed, stateVectorKin.x_d)

    stateVectorNonLinear_Base =  VehicleModel.dyn_model_base(stateVectorNonLinear_Base, carControl, dt)
    dyn_base_psi = vcat(dyn_base_psi, stateVectorNonLinear_Base.psi)

    stateVectorNonLinear_Enhanced_Lat =  VehicleModel.dyn_model_enhanced_lat(stateVectorNonLinear_Enhanced_Lat, carControl, dt)
    dyn_lat_psi = vcat(dyn_lat_psi, stateVectorNonLinear_Enhanced_Lat.psi)

    stateVectorNonLinear_Enhanced_Lat_Cmplx =  VehicleModel.dyn_model_enhanced_lat_cmplx(stateVectorNonLinear_Enhanced_Lat_Cmplx, carControl, dt)
    dyn_lat_cplx_psi = vcat(dyn_lat_cplx_psi, stateVectorNonLinear_Enhanced_Lat_Cmplx.psi)

end


print("size of kin_psi", length(kin_psi))
print("max_speed", max_speed[indmax(max_speed)])

lin = linspace(1, max_steps, max_steps)

areas = 5*ones(lin)
subplot(211)
scatter(lin,controlVectorSteer,s=areas,alpha=1.0)
grid()
xlabel("Time Steps")
ylabel("Steering Wheel")
title("Driver Input")
ax = gca()
subplot(212)
xlabel("Time Steps")
ylabel("Psi in rad")
title("Orientation of Car")
scatter(lin,kin_psi,s=areas,alpha=1.0)
scatter(lin,dyn_base_psi,s=areas,alpha=1.0)
scatter(lin,dyn_lat_psi,s=areas,alpha=1.0)
scatter(lin,dyn_lat_cplx_psi,s=areas,alpha=1.0)

grid()
