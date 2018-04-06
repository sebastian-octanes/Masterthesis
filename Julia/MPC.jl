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

function initMPC(N_, dt, startPose, tangentPoints, midTrackPoints, trackPoints, printLevel)

     global m = Model(solver = IpoptSolver(print_level = printLevel, max_iter=5000))
     #global m = Model(solver = MosekSolver())

     global N = N_
     lbx = []
     ubx = []
     start = []
     trackWidth = 4
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
     #=for i in 0:N-1
          p1 = [y[i*8 + 1], y[i*8 + 2]]
          p2 = [y[i*8 + 3], y[i*8 + 4]]
          p3 = [y[i*8 + 5], y[i*8 + 6]]
          p4 = [y[i*8 + 7], y[i*8 + 8]]

          @NLconstraint(m, (x[(i+1)*6 + 1]-p1[1]) * (p2[2] - p1[2]) - (x[(i+1)*6 + 2] - p1[2]) * (p2[1]- p1[1]) >= 0)
          @NLconstraint(m, (x[(i+1)*6 + 1]-p3[1]) * (p4[2] - p3[2]) - (x[(i+1)*6 + 2] - p3[2]) * (p4[1]- p3[1]) <= 0)
     end
     =#

     global t = @NLparameter(m, t[i=1:N*6] == trackPoints[i])
#=
     for i in 0:N-1
          x0 = [t[i*6 + 1], t[i*6 + 2]]
          x1 = [t[i*6 + 3], t[i*6 + 4]]
          x2 = [t[i*6 + 5], t[i*6 + 6]]

          @NLexpression(m, ax, x[(i+1)*6 + 1] - x0[1])
          @NLexpression(m, ay, x[(i+1)*6 + 2] - x0[2])
          @NLexpression(m, ab1, ax*(x1[1] - x0[1])+ ay*(x1[2] - x0[2]))
          @NLexpression(m, B1, sqrt((x1[1] - x0[1])^2 + (x1[2] - x0[2])^2))
          @NLconstraint(m, ab1/B1 <= trackWidth/2)

          @NLexpression(m, ax, x[(i+1)*6 + 1] - x0[1])
          @NLexpression(m, ay, x[(i+1)*6 + 2] - x0[2])
          @NLexpression(m, ab2, ax*(x2[1] - x0[1])+ ay*(x2[2] - x0[2]))
          @NLexpression(m, B2, sqrt((x2[1] - x0[1])^2 + (x2[2] - x0[2])^2))
          @NLconstraint(m, ab2/B2 <= trackWidth/2)
     end
=#

     #add more or less soft constraint to keep the car inside the racecourse even if tangents arent enough. keep it as general as possible!
     global z = @NLparameter(m, z[i = 1:N*2] == midTrackPoints[i])
#=
     for i in 0:N-1
          p1 = [z[i*2 + 1], z[i*2 + 2]]
          @NLconstraint(m, sqrt((x[(i+1)*6 + 1] - p1[1])^2 +  (x[(i+1)*6 + 2] - p1[2])^2) <= trackWidth*1.0)
     end
=#
     #enforce starting point
     global startPosX = @constraint(m, startPosX, x[1] == startPose.x)
     global startPosY = @constraint(m, startPosY, x[2] == startPose.y)
     global startPosV = @constraint(m, startPosV, x[3] == startPose.v)
     global startPosYaw = @constraint(m, startPosYaw, x[4] == startPose.yaw)

     #objective
     #@NLobjective(m, Max, x[N*6 + 3])


     alpha = 1
     k1 = -trackWidth/2
     k2 = trackWidth/2
     alpha2 = 2/ (8*(trackWidth/2)^7)
     dist_val1(xX, xY, x0X, x0Y, x1X, x1Y) =  ((xX - x0X)*(x1X - x0X) + (xY - x0Y)*(x1Y - x0Y)) / sqrt((x1X - x0X)^2 + (x1Y - x0Y)^2)
     #efunc1(xX, xY, x0X, x0Y, x1X, x1Y) = exp(alpha*(k1 + dist_val1(xX, xY, x0X, x0Y, x1X, x1Y)) + exp(-alpha*(k2 + dist_val1(xX, xY, x0X, x0Y, x1X, x1Y))))
     #efunc1(xX, xY, x0X, x0Y, x1X, x1Y) = abs(alpha/(k1 - dist_val1(xX, xY, x0X, x0Y, x1X, x1Y)) + (alpha/(k2 - dist_val1(xX, xY, x0X, x0Y, x1X, x1Y))))
     efunc1(xX, xY, x0X, x0Y, x1X, x1Y) = alpha2 * dist_val1(xX, xY, x0X, x0Y, x1X, x1Y)^8

     dist_val2(xX, xY, x0X, x0Y, x2X, x2Y) =  ((xX - x0X)*(x2X - x0X) + (xY - x0Y)*(x2Y - x0Y)) / sqrt((x2X - x0X)^2 + (x2Y - x0Y)^2)
     efunc2(xX, xY, x0X, x0Y, x2X, x2Y) = exp(alpha*(k1 + dist_val2(xX, xY, x0X, x0Y, x2X, x2Y)) + exp(-alpha*(k2 + dist_val2(xX, xY, x0X, x0Y, x2X, x2Y))))
     #efunc2(xX, xY, x0X, x0Y, x2X, x2Y) = abs(alpha/(k1 - dist_val2(xX, xY, x0X, x0Y, x2X, x2Y)) + (alpha/(k2 - dist_val2(xX, xY, x0X, x0Y, x2X, x2Y))))

     JuMP.register(m, :dist_val1, 6, dist_val1, autodiff=true)
     JuMP.register(m, :efunc1, 6, efunc1, autodiff=true)
     JuMP.register(m, :dist_val2, 6, dist_val2, autodiff=true)
     JuMP.register(m, :efunc2, 6, efunc2, autodiff=true)
     @NLexpression(m, sum_1, sum(efunc1(x[(i+1)*6 + 1], x[(i+1)*6 + 2], t[i*6 + 1], t[i*6 + 2], t[i*6 + 3], t[i*6 + 4]) for i in 0:N-1))
     @NLexpression(m, sum_2, sum(efunc2(x[(i+1)*6 + 1], x[(i+1)*6 + 2], t[i*6 + 1], t[i*6 + 2], t[i*6 + 5], t[i*6 + 6]) for i in 0:N-1))

     @NLexpression(m, sum_3, sum(1/x[i*6 + 3] for i in 1:N))
     @NLobjective(m, Min, sum_1 + sum_3)
     #@NLobjective(m, Max, sum(x[i*6 + 3] for i in 1:N))

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

function updateTrackPoints(trackPoints)
     for i in 0:N-1
          setvalue(t[i*6 + 1] , trackPoints[i*6 + 1])
          setvalue(t[i*6 + 2] , trackPoints[i*6 + 2])
          setvalue(t[i*6 + 3] , trackPoints[i*6 + 3])
          setvalue(t[i*6 + 4] , trackPoints[i*6 + 4])
          setvalue(t[i*6 + 5] , trackPoints[i*6 + 5])
          setvalue(t[i*6 + 6] , trackPoints[i*6 + 6])
     end
     return m, t
end

function updateMidTrackPoints(midTrackPoints)
     for i in 0:N-1
          setvalue(z[i*2 + 1] , midTrackPoints[i*2 + 1])
          setvalue(z[i*2 + 2] , midTrackPoints[i*2 + 2])
     end
     return m, y
end
function solveMPC()
     solve(m)
     res = getvalue(x)
     return res
end

export solveMPC, updateTangetPoints, updateStartPoint, initMPC, updateMidTrackPoints, updateTrackPoints
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
