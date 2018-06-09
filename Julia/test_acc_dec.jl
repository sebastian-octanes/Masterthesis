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

#average acceleration
stateVectorKin= VehicleModel.CarPose(0,0,0.1,0, 0, 0)
#motor power and tire force
stateVectorNonLinear_Enhanced_Long = VehicleModel.CarPose(0,0,0.1,0, 0, 0)
#friction and air resistance
stateVectorNonLinear_Cplx_Long = VehicleModel.CarPose(0,0,0.1,0, 0, 0)




kin_long_speed = []
dyn_long_speed = []
dyn_long_cplx_speed = []

for i in 1:max_steps
    carControl = VehicleModel.CarControls(controlVectorThrottle[i], controlVectorSteer[i])

    stateVectorKin =  VehicleModel.kin_bycicle_model(stateVectorKin, carControl, dt)
    kin_long_speed = vcat(kin_long_speed, stateVectorKin.x_d)

    stateVectorNonLinear_Enhanced_Long =  VehicleModel.dyn_model_enhanced_long(stateVectorNonLinear_Enhanced_Long, carControl, dt)
    dyn_long_speed = vcat(dyn_long_speed, stateVectorNonLinear_Enhanced_Long.x_d)

    stateVectorNonLinear_Cplx_Long =  VehicleModel.dyn_model_cplx_long(stateVectorNonLinear_Cplx_Long, carControl, dt)
    dyn_long_cplx_speed = vcat(dyn_long_cplx_speed, stateVectorNonLinear_Cplx_Long.x_d)

end


#print("size of kin_psi", length(kin_psi))
print("max_speed", kin_long_speed[indmax(kin_long_speed)])

lin = linspace(1, max_steps, max_steps)
lin = lin*0.05

areas = 5*ones(lin)
#=
subplot(211)
scatter(lin,controlVectorThrottle,s=areas,alpha=1.0)
grid()
xlabel("Time Steps")
ylabel("Throttle Input")
title("Driver Input")


subplot(212)
=#

open("outputFiles/accdec.txt", "w") do io
    writedlm(io, [lin kin_long_speed dyn_long_speed  dyn_long_cplx_speed])
end

ax = gca()
xlabel("Time Steps")
ylabel("Speed in m/s")
title("Orientation of Car")

scatter(lin,dyn_long_speed,s=areas,alpha=1.0)
scatter(lin,kin_long_speed,s=areas,alpha=1.0)
scatter(lin,dyn_long_cplx_speed,s=areas,alpha=1.0)

grid()
