module MPC
#using Mosek
using JuMP
using Ipopt
#using KNITRO

include("VehicleModel.jl")
#using VehicleModel

#using Clp
#using NLopt
#using KNITRO
#using Mosek

function initMPC(N_, dt, startPose, tangentPoints, printLevel)

     global m = Model(solver = IpoptSolver(print_level = printLevel))
     #global m = Model(solver = MosekSolver())

     global N = N_
     lbx = []
     ubx = []
     start = []

     lbx_ = [-Inf, -Inf,                   0.01, -Inf, -VehicleModel.max_long_dec, -VehicleModel.max_steering_angle]
     ubx_ = [ Inf,  Inf, VehicleModel.max_speed,  Inf,  VehicleModel.max_long_acc,  VehicleModel.max_steering_angle]
     start_=[startPose.x, startPose.y, startPose.v, startPose.yaw, 0, 0]
     for i in 0:N
          lbx = vcat(lbx, lbx_) #add lower bounds to vector
          ubx = vcat(ubx, ubx_) #add upper bounds to vector
          start = vcat(start, start_) #add initial guess to vector
     end

     global x = @variable(m, lbx[i] <= x[i = 1:(N+1)*6] <= ubx[i], start = start[i]) #set bounds and initial guess and create x-vector

     lf = VehicleModel.lf
     lr = VehicleModel.lr

     #create vehicle model constraints
     for i in 0: N-1
          @NLconstraints(m, begin
               x[(i + 1)*6 + 1] - (x[i * 6 + 1] + x[6*i + 3]*dt*cos(x[i*6 + 4] + atan(lr/(lf + lr) * tan(x[i*6 + 6])))) == 0
               x[(i + 1)*6 + 2] - (x[i * 6 + 2] + x[6*i + 3]*dt*sin(x[i*6 + 4] + atan(lr/(lf + lr) * tan(x[i*6 + 6])))) == 0
               x[(i + 1)*6 + 3] - (x[i * 6 + 3] + x[6*i + 5]*dt) == 0
               x[(i + 1)*6 + 4] - (x[i * 6 + 4] + x[6*i + 3]*dt / lr*sin(atan(lr/(lf + lr) * tan(x[i*6 + 6])))) == 0
               atan(0.5 * (lf + lr) * VehicleModel.max_long_acc / x[i*6 + 3]^2) - atan(lr/(lf + lf) * tan(x[i*6 + 6])) >= 0  #max_beta - beta
               atan(0.5 * (lf + lr) * VehicleModel.max_long_acc / x[i*6 + 3]^2) + atan(lr/(lf + lf) * tan(x[i*6 + 6])) >= 0  #max_beta + beta
          end)
     end

     global y = @NLparameter(m, y[i=1:N*4*2] == tangentPoints[i])
     for i in 0:N-1
          p1 = [y[i*8 + 1], y[i*8 + 2]]
          p2 = [y[i*8 + 3], y[i*8 + 4]]
          p3 = [y[i*8 + 5], y[i*8 + 6]]
          p4 = [y[i*8 + 7], y[i*8 + 8]]

          @NLconstraint(m, (x[(i+1)*6 + 1]-p1[1]) * (p2[2] - p1[2]) - (x[(i+1)*6 + 2] - p1[2]) * (p2[1]- p1[1]) >= 0)
          @NLconstraint(m, (x[(i+1)*6 + 1]-p3[1]) * (p4[2] - p3[2]) - (x[(i+1)*6 + 2] - p3[2]) * (p4[1]- p3[1]) <= 0)
     end



     #enforce starting point
     global startPosX = @constraint(m, startPosX, x[1] == startPose.x)
     global startPosY = @constraint(m, startPosY, x[2] == startPose.y)
     global startPosV = @constraint(m, startPosV, x[3] == startPose.v)
     global startPosYaw = @constraint(m, startPosYaw, x[4] == startPose.yaw)

     #objective
     #@NLobjective(m, Max, x[N*6 + 3])
     @NLobjective(m, Max, sum(x[i*6 + 3] for i in 1:N))
     return m
end

function updateStartPoint(res)
     #enforce starting point
     JuMP.setRHS(startPosX, res[1])
     JuMP.setRHS(startPosY, res[2])
     JuMP.setRHS(startPosV, res[3])
     JuMP.setRHS(startPosYaw, res[4])
end

function updateTangentPoints(tangetPoints)
     for i in 0:N-1
          setvalue(y[i*8 + 1] , tangetPoints[i*8 + 1])
          setvalue(y[i*8 + 2] , tangetPoints[i*8 + 2])
          setvalue(y[i*8 + 3] , tangetPoints[i*8 + 3])
          setvalue(y[i*8 + 4] , tangetPoints[i*8 + 4])
          setvalue(y[i*8 + 5] , tangetPoints[i*8 + 5])
          setvalue(y[i*8 + 6] , tangetPoints[i*8 + 6])
          setvalue(y[i*8 + 7] , tangetPoints[i*8 + 7])
          setvalue(y[i*8 + 8] , tangetPoints[i*8 + 8])
     end
     return m, y
end
function solveMPC()
     solve(m)
     res = getvalue(x)
     return res
end

export solveMPC, updateTangetPoints, updateStartPoint, initMPC
end

#=
include("vehiclemodel.jl")
using VehicleModel
using MPC

N = 1
dt = 0.01
tangentPoints =[0,0,1,3,4,2,4,5]
startPose = VehicleModel.CarPose(0,0,0.01,pi/2)
initMPC(N, dt, startPose, tangentPoints)
println(m)
solveMPC()
println("x = ", getvalue(x))
tangentPoints =[0,-1,1,3,4,2,4,5]
updateTangetPoints(tangentPoints)
solveMPC()
=#
