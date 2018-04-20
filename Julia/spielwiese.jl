include("smallMPC.jl")
using PyPlot
using Distributions
#mpc_struct = MPCStruct(0,0,0)
#mpc_struct = init_MPC(mpc_struct)
#mpc_struct = defineVariables(mpc_struct)
#mpc_struct = define_Objective(mpc_struct)
#print_mpc(mpc_struct)
#solve_mpc(mpc_struct)

start_ = 0.01
end_ = 0.21
steps_ = 5
spline_pos =[]
#lin = linspace(start_,end_,steps_)
lin = linspace(-1, 1, 101)
#lin = [0.05, 0.01, 0.05, 0.06]
print(lin)
for i in lin
    spline_pos = vcat(spline_pos, i)
    print("\ni ", i)
end

areas = 10*ones(lin)
scatter(lin,spline_pos,s=areas,alpha=1.0)
