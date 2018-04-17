include("smallMPC.jl")
using PyPlot
#mpc_struct = MPCStruct(0,0,0)
#mpc_struct = init_MPC(mpc_struct)
#mpc_struct = defineVariables(mpc_struct)
#mpc_struct = define_Objective(mpc_struct)
#print_mpc(mpc_struct)
#solve_mpc(mpc_struct)


t= 3

x = linspace(10, 10 + t -1, t); y = sin.(3 * x + 4 * cos.(2 * x));
areas = 10* rand(x)
#plot(x, y, color="red", linewidth=2.0, linestyle="--")
scatter(x,y,s=areas,alpha=0.5)
xticks(x)
title("A sinusoidally modulated sinusoid")
