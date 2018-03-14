using JuMP
#using Clp
#using Ipopt
#using NLopt
#using KNITRO
using Mosek

N = 2
dt = 0.1
lr = 1
lf = 1.5
max_acc = 9
m = Model(solver = MosekSolver())
@variable(m, x[1:(N+1) *6] )
lbx = []
ubx = []
start = []
lbx_ = [-Inf, -Inf, 0.01, -Inf, -9, -30*pi/180]
ubx_ = [ Inf,  Inf,   30,  Inf,  9,  30*pi/180]
start_=[0, 0, 0.01, pi/2, 0, 0]
for i in 0 : N
     lbx = vcat(lbx, lbx_)
     ubx = vcat(ubx, ubx_)
     start = vcat(start, start_)
end
#set bounds
@variable(m, x[i = 1:(N+1)*6] >= lbx[i])
@variable(m, x[i = 1:(N+1)*6] <= ubx[i])

#set start value
@variable(m , x[i = 1:(N+1)*6], start = start[i])

i = 0
for i in 0: N-1
     @NLconstraints(m, begin
          x[(i + 1)*6 + 1] - (x[i * 6 + 1] + x[6*i + 3]*dt*cos(i*6 + 4) + atan(lr/(lf + lf) * tan(x[i*6 + 6]))) == 0
          x[(i + 1)*6 + 2] - (x[i * 6 + 2] + x[6*i + 3]*dt*sin(i*6 + 4) + atan(lr/(lf + lf) * tan(x[i*6 + 6]))) == 0
          x[(i + 1)*6 + 3] - (x[i * 6 + 3] + x[6*i + 5]*dt) == 0
          x[(i + 1)*6 + 4] - (x[i * 6 + 4] + x[6*i + 3]*dt / lr*sin(atan(lr/(lf + lf) * tan(x[i*6 + 6])))) == 0
          atan(0.5 * (lf + lr) * max_acc / x[i*6 + 4]^2) - atan(lr/(lf + lf) * tan(x[i*6 + 6])) == 0  #max_beta - beta
          atan(0.5 * (lf + lr) * max_acc / x[i*6 + 4]^2) + atan(lr/(lf + lf) * tan(x[i*6 + 6])) == 0  #max_beta + beta

     end)
end

@objective(m, Min, x[(N)*6 + 1] + x[(N)*6 + 2])#sqrt(x[(N-1)*6 + 1]^2 + x[(N-1)*6 + 2]^2))

println(m)
solve(m)
#println("x = ", getvalue(x))

#=
println("Objective value: ", getobjectivevalue(m))
println("x = ", getvalue(x))
println("y = ", getvalue(y))
=#
