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
m = Model(solver = IpoptSolver())
#@variable(m, x[1:(N+1) *3] )
println(m)
lbx = []
ubx = []
start = []
lbx_ = [-Inf, -Inf, 0.01, -Inf, -9, -30*pi/180]
ubx_ = [ Inf,  Inf,   30,  Inf,  9,  30*pi/180]
start_=[0, 0, 0.1, pi/2, 0, 0]
for i in 1 : N+1
     lbx = vcat(lbx, lbx_)
     ubx = vcat(ubx, ubx_)
     start = vcat(start, start_)
end #set bounds


@variable(m, lbx[i] <= x[i = 1:(N+1)*6] <= ubx[i], start = start[i])

for i in 0: N-1
     @NLconstraints(m, begin
          x[(i + 1)*6 + 1] - (x[i * 6 + 1] + x[6*i + 3]*dt*cos(x[i*6 + 4]) + atan(lr/(lf + lr) * tan(x[i*6 + 6]))) == 0
          x[(i + 1)*6 + 2] - (x[i * 6 + 2] + x[6*i + 3]*dt*sin(x[i*6 + 4]) + atan(lr/(lf + lr) * tan(x[i*6 + 6]))) == 0
          x[(i + 1)*6 + 3] - (x[i * 6 + 3] + x[6*i + 5]*dt) == 0
          x[(i + 1)*6 + 4] - (x[i * 6 + 4] + x[6*i + 3]*dt / lr*sin(atan(lr/(lf + lr) * tan(x[i*6 + 6])))) == 0
#          x[(i + 1)*3 + 1] - (x[i * 3 + 1] + x[3*i + 2]*dt) == 0
#          x[(i + 1)*3 + 2] - (x[i * 3 + 2] + x[3*i + 3]*dt) == 0
     end)
end

@objective(m, Max, x[(N)*6 + 3])#sqrt(x[(N-1)*6 + 1]^2 + x[(N-1)*6 + 2]^2))

#@NLparameter(m, x[1] == 0)
#@NLparameter(m, x[2] == 0.1)
@constraint(m, x[1] == 0)
@constraint(m, x[2] == 0)
@constraint(m, x[3] == 0.01)
@constraint(m, x[4] == pi/2)



println(m)
solve(m)
println("x = ", getvalue(x))

#=
println("Objective value: ", getobjectivevalue(m))
println("x = ", getvalue(x))
println("y = ", getvalue(y))
=#
