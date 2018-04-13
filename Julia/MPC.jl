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
     global N = N_


     lbx = []
     ubx = []
     start = []
     trackWidth = 4
     lbx_ = [-Inf, -Inf,                   0.01, -Inf, -Inf, -Inf,  -VehicleModel.max_long_dec, -VehicleModel.max_steering_angle]
     ubx_ = [ Inf,  Inf, VehicleModel.max_speed,  Inf,  Inf,  Inf,   VehicleModel.max_long_acc,  VehicleModel.max_steering_angle]
     start_=[startPose.x, startPose.y, startPose.x_d, startPose.psi, 0, 0, 0, 0]
     for i in 0:N
          lbx = vcat(lbx, lbx_) #add lower bounds to vector
          ubx = vcat(ubx, ubx_) #add upper bounds to vector
          start = vcat(start, start_) #add initial guess to vector
     end

     global x = @variable(m, lbx[i] <= x[i = 1:(N+1)*8] <= ubx[i], start = start[i]) #set bounds and initial guess and create x-vector

     lf = VehicleModel.lf
     lr = VehicleModel.lr

#=
     #create vehicle model constraints
     for i in 0: N-1
          @NLconstraints(m, begin
               x[(i + 1)*8 + 1] - (x[i * 8 + 1] + x[8*i + 3]*dt*cos(x[i*8 + 4] + atan(lr/(lf + lr) * tan(x[i*8 + 8])))) == 0
               x[(i + 1)*8 + 2] - (x[i * 8 + 2] + x[8*i + 3]*dt*sin(x[i*8 + 4] + atan(lr/(lf + lr) * tan(x[i*8 + 8])))) == 0
               x[(i + 1)*8 + 3] - (x[i * 8 + 3] + x[8*i + 7]*dt) == 0
               x[(i + 1)*8 + 4] - (x[i * 8 + 4] + x[8*i + 3]*dt / lr*sin(atan(lr/(lf + lr) * tan(x[i*8 + 8])))) == 0
               atan(0.5 * (lf + lr) * VehicleModel.max_long_acc / x[i*8 + 3]^2) - atan(lr/(lf + lf) * tan(x[i*8 + 8])) >= 0  #max_beta - beta
               atan(0.5 * (lf + lr) * VehicleModel.max_long_acc / x[i*8 + 3]^2) + atan(lr/(lf + lf) * tan(x[i*8 + 8])) >= 0  #max_beta + beta
          end)
     end
=#



     #create vehicle model constraints
     for i in 0: N-1

          #expression for tire model
          @NLexpression(m, slip_angle_f, x[i * 8 + 8] - atan((x[i * 8 + 5]+ VehicleModel.lf * x[i * 8 + 6]) / x[i * 8 + 3]))
          @NLexpression(m, slip_angle_b,              - atan((x[i * 8 + 5]- VehicleModel.lf * x[i * 8 + 6]) / x[i * 8 + 3]))

          @NLexpression(m, Fbx, VehicleModel.F_long_max * x[8*i + 7]/10.0)
          @NLexpression(m, Ffy, VehicleModel.Df * slip_angle_f / VehicleModel.xmf )
          @NLexpression(m, Fby, VehicleModel.Db * slip_angle_b / VehicleModel.xmb )

          @NLconstraints(m, begin


               x[(i + 1)*8 + 1] - (x[i * 8 + 1] + dt * (x[8*i + 3]*cos(x[i*8 + 4]) - x[8*i + 5] * sin(x[8*i + 4]))) == 0

               x[(i + 1)*8 + 2] - (x[i * 8 + 2] + dt * (x[8*i + 3]*sin(x[i*8 + 4]) + x[8*i + 5] * cos(x[8*i + 4]))) == 0

               x[(i + 1)*8 + 3] - (x[i * 8 + 3] + dt * (Fbx - Ffy * sin(x[i * 8 + 8]) + VehicleModel.mass * x[i * 8 + 5] * x[i * 8 + 6]) * (1.0/VehicleModel.mass)) == 0

               x[(i + 1)*8 + 4] - (x[i * 8 + 4] + dt*(x[8*i + 6])) == 0

               x[(i + 1)*8 + 5] - (x[i*8 + 5] + dt * (Fby + Ffy * cos(x[8*i + 8]) - VehicleModel.mass * x[8*i + 3] * x[8*i + 6])* (1.0/VehicleModel.mass)) == 0

               x[(i + 1)*8 + 6] - (x[i*8 + 6] + dt * (VehicleModel.lf * Ffy * cos(x[i*8 + 8]) - VehicleModel.lr * Fby)/VehicleModel.I) == 0

          end)
     end

#=
     #create vehicle model constraints
     for i in 0: N-1
          #expression for Aero constraint
          #@NLexpression(m, Faero,  1/2.0 * VehicleModel.rho * VehicleModel.Cd * VehicleModel.Af * x[(i+1)*8 + 3]^2)
          #expression for Friction constraint
          #@NLexpression(m, Rxf,  VehicleModel.mu * VehicleModel.mass * VehicleModel.g * VehicleModel.lf / (VehicleModel.lf + VehicleModel.lr))
          #@NLexpression(m, Rxb,  VehicleModel.mu * VehicleModel.mass * VehicleModel.g * VehicleModel.lr / (VehicleModel.lf + VehicleModel.lr))
          #@expression(m, Fbx, VehicleModel.F_long_max * x[8*i + 7]/10.0 - Faero - Rxf - Rxb)
          @NLexpression(m, Fbx, VehicleModel.F_long_max * x[8*i + 7]/10.0)

          #expression for tire model
          #slip_angle_f = phi- atan((y_d + lf * psi_d)/ x_d)
          @NLexpression(m, slip_angle_f, x[i * 8 + 7] - atan((x[i * 8 + 5]+ VehicleModel.lf * x[i * 8 + 6]) / x[i * 8 + 3]))
          #slip_angle_b = - atan((y_d - lf * psi_d)/ x_d)
          @NLexpression(m, slip_angle_b,              - atan((x[i * 8 + 5]- VehicleModel.lf * x[i * 8 + 6]) / x[i * 8 + 3]))
          #linear tire model
          @NLexpression(m, Ffy, VehicleModel.Df * slip_angle_f / VehicleModel.xmf )
          @NLexpression(m, Fby, VehicleModel.Db * slip_angle_b / VehicleModel.xmb )

          @NLconstraints(m, begin
               #x constraint
               #x+1 = x + dt * (x_d * cos(psi) - y_d *sin(psi))
               #x+1 - (x + dt *(x_d * cos(psi) - y_d *sin(psi))) == 0
               x[(i + 1)*8 + 1] - (x[i * 8 + 1] + dt * (x[8*i + 3]*cos(x[i*8 + 4]) - x[8*i + 5] * sin(x[8*i + 4]))) == 0

               #y constraint
               #y+1 = y + dt * (x_d * sin(psi) + y_d *cos(psi))
               #y+1 - (y + dt *(x_d * sin(psi) + y_d *cos(psi))) == 0
               x[(i + 1)*8 + 2] - (x[i * 8 + 2] + dt * (x[8*i + 3]*sin(x[i*8 + 4]) + x[8*i + 5] * cos(x[8*i + 4]))) == 0

               #constraints for longitudinal movement
               #x_d+1 = x_d + dt * (Fbx - Ffy * sin(phi) + mass * y_d * psi_d)*(1.0/mass)
               #x_d+1 - (x_d + dt * (Fbx - Ffy * sin(phi) + mass * y_d * psi_d) * (1.0/mass)) == 0
               x[(i + 1)*8 + 3] - (x[i * 8 + 3] + dt * (Fbx - Ffy * sin(x[i * 8 + 8]) + VehicleModel.mass * x[i * 8 + 5] * x[i * 8 + 6]) * (1.0/VehicleModel.mass)) == 0

               #constraint for yaw
               #psi+1 = psi + dt * psi_d
               #psi+1 -(psi + dt*(psi_d)) == 0
               x[(i + 1)*8 + 4] - (x[i * 8 + 4] + dt*(x[8*i + 6])) == 0

               #constraint for lateral movement
               #y_d+1 = y_d + dt * (Fby + Ffy * cos(phi) - mass * x_d * psi_d)*(1.0 / mass)
               #y_d+1 - (y_d + dt * (Fby + Ffy * cos(phi) - mass * x_d * psi_d)*(1.0 / mass)) == 0
               x[(i + 1)*8 + 5] - (x[i*8 + 5] + dt * (Fby + Ffy * cos(x[8*i + 8]) - VehicleModel.mass * x[8*i + 3] * x[8*i + 6])* (1.0/VehicleModel.mass)) == 0

               #constraint for yaw rate
               #psi_d+1 = psi_d + dt * (lf*Ffy*cos(phi) - lr*Fby)/I
               #psi_d+1 - psi_d + dt * (lf * Ffy * cos(phi) - lr*Fby)/I) == 0
               x[(i + 1)*8 + 6] - (x[i*8 + 6] + dt * (VehicleModel.lf * Ffy * cos(x[i*8 + 8]) - VehicleModel.lr * Fby)/VehicleModel.I) == 0

          end)
     end

=#

     global y = @NLparameter(m, y[i=1:N*4*2] == tangentPoints[i])
     #=
     for i in 0:N-1
          p1 = [y[i*8 + 1], y[i*8 + 2]]
          p2 = [y[i*8 + 3], y[i*8 + 4]]
          p3 = [y[i*8 + 5], y[i*8 + 6]]
          p4 = [y[i*8 + 7], y[i*8 + 8]]

          @NLconstraint(m, (x[(i+1)*8 + 1]-p1[1]) * (p2[2] - p1[2]) - (x[(i+1)*8 + 2] - p1[2]) * (p2[1]- p1[1]) >= 0)
          @NLconstraint(m, (x[(i+1)*8 + 1]-p3[1]) * (p4[2] - p3[2]) - (x[(i+1)*8 + 2] - p3[2]) * (p4[1]- p3[1]) <= 0)
     end

=#
     global t = @NLparameter(m, t[i=1:N*6] == trackPoints[i])

     for i in 0:N-1
          x0 = [t[i*6 + 1], t[i*6 + 2]]
          x1 = [t[i*6 + 3], t[i*6 + 4]]
          x2 = [t[i*6 + 5], t[i*6 + 6]]

          @NLexpression(m, ax, x[(i+1)*8 + 1] - x0[1])
          @NLexpression(m, ay, x[(i+1)*8 + 2] - x0[2])
          @NLexpression(m, ab1, ax*(x1[1] - x0[1])+ ay*(x1[2] - x0[2]))
          @NLexpression(m, B1, sqrt((x1[1] - x0[1])^2 + (x1[2] - x0[2])^2))
          @NLconstraint(m, ab1/B1 <= trackWidth/2)

          @NLexpression(m, ax, x[(i+1)*8 + 1] - x0[1])
          @NLexpression(m, ay, x[(i+1)*8 + 2] - x0[2])
          @NLexpression(m, ab2, ax*(x2[1] - x0[1])+ ay*(x2[2] - x0[2]))
          @NLexpression(m, B2, sqrt((x2[1] - x0[1])^2 + (x2[2] - x0[2])^2))
          @NLconstraint(m, ab2/B2 <= trackWidth/2)
     end

     #add more or less soft constraint to keep the car inside the racecourse even if tangents arent enough. keep it as general as possible!
     global z = @NLparameter(m, z[i = 1:N*2] == midTrackPoints[i])


     for i in 0:N-1
          p1 = [z[i*2 + 1], z[i*2 + 2]]

          @NLconstraint(m, sqrt((x[(i+1)*8 + 1] - p1[1])^2 +  (x[(i+1)*8 + 2] - p1[2])^2) <= trackWidth*0.5)
     end


     #enforce starting point
     global startPosX = @constraint(m, startPosX, x[1] == startPose.x)
     global startPosY = @constraint(m, startPosY, x[2] == startPose.y)
     global startPosX_d = @constraint(m, startPosX_d, x[3] == startPose.x_d)
     global startPosPsi = @constraint(m, startPosPsi, x[4] == startPose.psi)
     global startPosY_d = @constraint(m, startPosY_d, x[5] == startPose.y_d)
     global startPosPsi_d = @constraint(m, startPosPsi_d, x[6] == startPose.psi_d)

     #objective
#=

     alpha = 0.00002
     k1 = -trackWidth/0.5
     k2 = trackWidth/0.5
     alpha2 = 2/ (8*(trackWidth/4)^7)
     dist_val1(xX, xY, x0X, x0Y, x1X, x1Y) =  ((xX - x0X)*(x1X - x0X) + (xY - x0Y)*(x1Y - x0Y)) / sqrt((x1X - x0X)^2 + (x1Y - x0Y)^2)
     #efunc1(xX, xY, x0X, x0Y, x1X, x1Y) = exp(alpha*(k1 + dist_val1(xX, xY, x0X, x0Y, x1X, x1Y)) + exp(-alpha*(k2 + dist_val1(xX, xY, x0X, x0Y, x1X, x1Y))))
     #efunc1(xX, xY, x0X, x0Y, x1X, x1Y) = abs(alpha/(k1 - dist_val1(xX, xY, x0X, x0Y, x1X, x1Y)) + (alpha/(k2 - dist_val1(xX, xY, x0X, x0Y, x1X, x1Y))))
     efunc1(xX, xY, x0X, x0Y, x1X, x1Y) = alpha2 * dist_val1(xX, xY, x0X, x0Y, x1X, x1Y)^2

     dist_val2(xX, xY, x0X, x0Y, x2X, x2Y) =  ((xX - x0X)*(x2X - x0X) + (xY - x0Y)*(x2Y - x0Y)) / sqrt((x2X - x0X)^2 + (x2Y - x0Y)^2)
     efunc2(xX, xY, x0X, x0Y, x2X, x2Y) = exp(alpha*(k1 + dist_val2(xX, xY, x0X, x0Y, x2X, x2Y)) + exp(-alpha*(k2 + dist_val2(xX, xY, x0X, x0Y, x2X, x2Y))))
     #efunc2(xX, xY, x0X, x0Y, x2X, x2Y) = abs(alpha/(k1 - dist_val2(xX, xY, x0X, x0Y, x2X, x2Y)) + (alpha/(k2 - dist_val2(xX, xY, x0X, x0Y, x2X, x2Y))))
     #efunc2(xX, xY, x0X, x0Y, x2X, x2Y) = alpha2 * dist_val2(xX, xY, x0X, x0Y, x2X, x2Y)^8

     JuMP.register(m, :dist_val1, 6, dist_val1, autodiff=true)
     JuMP.register(m, :efunc1, 6, efunc1, autodiff=true)
     JuMP.register(m, :dist_val2, 6, dist_val2, autodiff=true)
     JuMP.register(m, :efunc2, 6, efunc2, autodiff=true)
     @NLexpression(m, sum_1, sum(efunc1(x[(i+1)*8 + 1], x[(i+1)*8 + 2], t[i*6 + 1], t[i*6 + 2], t[i*6 + 3], t[i*6 + 4]) for i in 0:10))
     #@NLexpression(m, sum_2, sum(efunc2(x[(i+1)*8 + 1], x[(i+1)*8 + 2], t[i*6 + 1], t[i*6 + 2], t[i*6 + 5], t[i*6 + 6]) for i in 0:N-1))

     @NLexpression(m, sum_3, sum(1/x[i*8 + 3] for i in 1:N))
     @NLobjective(m, Min, sum_1 + sum_3)

=#
     @NLobjective(m, Max, sum(x[i*8 + 3] for i in 1:N))

     return m
end


function updateStartPointFromPose(carPose)
     #enforce starting point
     JuMP.setRHS(startPosX, carPose.x)
     JuMP.setRHS(startPosY, carPose.y)
     JuMP.setRHS(startPosX_d, carPose.x_d)
     JuMP.setRHS(startPosPsi, carPose.psi)
     JuMP.setRHS(startPosY_d, carPose.y_d)
     JuMP.setRHS(startPosPsi_d, carPose.psi_d)
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
     #x0 = [t[i*6 + 1], t[i*6 + 2]] /midpoint
     #x1 = [t[i*6 + 3], t[i*6 + 4]] / left bound
     #x2 = [t[i*6 + 5], t[i*6 + 6]] / right bound

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

export solveMPC, updateStartPointFromPose, updateTangetPoints, updateStartPoint, initMPC, updateMidTrackPoints, updateTrackPoints
end
