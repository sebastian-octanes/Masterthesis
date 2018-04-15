include("smallMPC.jl")

#mpc_struct = MPCStruct(0,0,0)
#mpc_struct = init_MPC(mpc_struct)
#mpc_struct = defineVariables(mpc_struct)
#mpc_struct = define_Objective(mpc_struct)
#print_mpc(mpc_struct)
#solve_mpc(mpc_struct)

startPoint = []
for i in 1:10
    startPoint = vcat(startPoint, i, i+20)
end
s = length(startPoint)
for i in 1:2:s
    print("\n i: ",i)
end


print("\n trackVehicleControls", trackVehicleControls[i], trackVehicleControls[i + 1])
carControl = VehicleModel.CarControls(1, 2)
#stateVectorLinear =  VehicleModel.linear_bycicle_model(stateVectorLinear, carControl, dt)
