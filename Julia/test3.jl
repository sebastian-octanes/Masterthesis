using JuMP
using Clp
using PyPlot
using Ipopt
#using NLopt
#using KNITRO
#using Mosek

N = 5
dt = 0.1
lr = 1
lf = 1.5
max_acc = 9
m = Model(solver = IpoptSolver())

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
end

@variable(m, lbx[i] <= x[i = 1:(N+1)*6] <= ubx[i], start = start[i])

for i in 0: N-1
     @NLconstraints(m, begin
          x[(i + 1)*6 + 1] - (x[i * 6 + 1] + x[6*i + 3]*dt*cos(x[i*6 + 4] + atan(lr/(lf + lr) * tan(x[i*6 + 6])))) == 0
          x[(i + 1)*6 + 2] - (x[i * 6 + 2] + x[6*i + 3]*dt*sin(x[i*6 + 4] + atan(lr/(lf + lr) * tan(x[i*6 + 6])))) == 0
          x[(i + 1)*6 + 3] - (x[i * 6 + 3] + x[6*i + 5]*dt) == 0
          x[(i + 1)*6 + 4] - (x[i * 6 + 4] + x[6*i + 3]*dt / lr*sin(atan(lr/(lf + lr) * tan(x[i*6 + 6])))) == 0
          atan(0.5 * (lf + lr) * max_acc / x[i*6 + 3]^2) - atan(lr/(lf + lf) * tan(x[i*6 + 6])) >= 0  #max_beta - beta
          atan(0.5 * (lf + lr) * max_acc / x[i*6 + 3]^2) + atan(lr/(lf + lf) * tan(x[i*6 + 6])) >= 0  #max_beta + beta
     end)
end




points2 = [1, 0 ,2, 1, 4, 0, 5, 1]
@NLparameter(m, y[i=1:8] == points2[i])
p1 = [y[1], y[2]]
p2 = [y[3], y[4]]
p3 = [y[5], y[6]]
p4 = [y[7], y[8]]

for i in 1:N-1
     @NLconstraint(m, (x[(i+1)*6 + 1]-p1[1]) * (p2[2] - p1[2]) - (x[(i+1)*6 + 2] - p1[2]) * (p2[1]- p1[1]) >= 0)
     @NLconstraint(m, (x[(i+1)*6 + 1]-p3[1]) * (p4[2] - p3[2]) - (x[(i+1)*6 + 2] - p3[2]) * (p4[1]- p3[1]) <= 0)
end



@constraint(m, startPosX, x[1] == 2)
@constraint(m, startPosY, x[2] == 0)
@constraint(m, startPosV, x[3] == 0.01)
@constraint(m, startPosYaw, x[4] == pi/2)


@NLobjective(m, Max, x[N*6 + 3])#sqrt(x[(N-1)*6 + 1]^2 + x[(N-1)*6 + 2]^2))
#@NLobjective(m, Max, x[(N-1)*6 + 1] + x[(N-1)*6 + 2])

println(m)

#print stuff
prin_x = []
prin_y = []


for i in 1:10
     solve(m)
     println("x = ", getvalue(x))
     res = getvalue(x)
     push!(prin_x, res[1])
     push!(prin_y, res[2])
     JuMP.setRHS(startPosX, res[7])
     JuMP.setRHS(startPosY, res[8])
     JuMP.setRHS(startPosV, res[9])
     JuMP.setRHS(startPosYaw, res[10])
end

#=
for i in 0:N
     a = 6*i +1
     b = 6*i +2
     push!(prin_x, res[a])
     push!(prin_y, res[b])
end
=#

plot(prin_x, prin_y)
#=
println("Objective value: ", getobjectivevalue(m))
println("x = ", getvalue(x))
println("y = ", getvalue(y))
=#
