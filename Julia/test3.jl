using JuMP
using Clp
using Ipopt
#using NLopt
#using KNITRO
#using Mosek

N = 1
dt = 0.1
lr = 1
lf = 1.5
max_acc = 9
m = Model(solver = ClpSolver())
@variable(m, x[1:(N+1) *3] )
println(m)
lbx = []
ubx = []
start = []
lbx_ = [ 0, 0.01, -9]
ubx_ = [ Inf,   30,  9]
start_=[0, 0.1,  0]
for i in 1 : N+1
     lbx = vcat(lbx, lbx_)
     ubx = vcat(ubx, ubx_)
     start = vcat(start, start_)
end #set bounds
@variable(m, x[i = 1:(N+1)*3] >= lbx[i])
@variable(m, x[i = 1:(N+1)*3] <= ubx[i])

#set start value
@variable(m , x[i = 1:(N+1)*3], start = start[i])

for i in 0: N-1
     @constraints(m, begin
          x[(i + 1)*3 + 1] - (x[i * 3 + 1] + x[3*i + 2]*dt) == 0
          x[(i + 1)*3 + 2] - (x[i * 3 + 2] + x[3*i + 3]*dt) == 0
     end)
end

@objective(m, Max, x[(N)*3 + 1])#sqrt(x[(N-1)*6 + 1]^2 + x[(N-1)*6 + 2]^2))

#@NLparameter(m, x[1] == 0)
#@NLparameter(m, x[2] == 0.1)
@constraint(m, x[1] == 0)
@constraint(m, x[2] == 0.01)

println(m)
solve(m)
println("x = ", getvalue(x))

#=
println("Objective value: ", getobjectivevalue(m))
println("x = ", getvalue(x))
println("y = ", getvalue(y))
=#
