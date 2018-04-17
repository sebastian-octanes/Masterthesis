using JuMP
using Ipopt

include("VehicleModel.jl")

mutable struct MPCStruct
    N #amount of steps
    m   #mpc model
    x   #state vector
    #constraint parameter
    startPose  #startpose
    t     #tangential points and midtrackPoints
    z     #point ahead of search vector to minimize distance to it
end


function init_MPC(mpc_struct, N_, dt, startPose, printLevel)
     m = Model(solver = IpoptSolver(tol=1e-1, print_level = printLevel, max_iter=500))
     N = N_

     lbx = []
     ubx = []
     start = []
     trackWidth = 4
     lbx_ = [-Inf, -Inf,                   0.01, -Inf, -Inf, -Inf,  -VehicleModel.max_long_dec, -VehicleModel.max_steering_angle]
     ubx_ = [ Inf,  Inf, VehicleModel.max_speed,  Inf,  Inf,  Inf,   VehicleModel.max_long_acc,  VehicleModel.max_steering_angle]
     start_=[startPose.x, startPose.y, startPose.x_d, startPose.psi, startPose.y_d, startPose.psi_d, 0, 0]
     for i in 0:N
          lbx = vcat(lbx, lbx_) #add lower bounds to vector
          ubx = vcat(ubx, ubx_) #add upper bounds to vector
          start = vcat(start, start_) #add initial guess to vector
     end

     x = @variable(m, lbx[i] <= x[i = 1:(N+1)*8] <= ubx[i], start = start[i]) #set bounds and initial guess and create x-vector
     #defined in init to make change of objective functions possible without changing mpc_struct
     #call setforwardPoint before using the objective function
     forwardDummy =[0,0]
     z = @NLparameter(m, z[i=1:2] == forwardDummy[i])
     mpc_struct.z = z

     mpc_struct.m = m
     mpc_struct.x = x
     mpc_struct.N = N
    return mpc_struct
end

function define_constraint_linear_bycicle(mpc_struct)
    x = mpc_struct.x
    m = mpc_struct.m
    N = mpc_struct.N

    lf = VehicleModel.lf
    lr = VehicleModel.lr
    #create vehicle model constraints
    for i in 0: N-1
         @NLexpression(m, x_d, (VehicleModel.F_long_max * x[8*i + 7]/10.0)* 1.0/VehicleModel.mass)
         @NLconstraints(m, begin
              x[(i + 1)*8 + 1] - (x[i * 8 + 1] + x[8*i + 3]*dt*cos(x[i*8 + 4] + atan(lr/(lf + lr) * tan(x[i*8 + 8])))) == 0
              x[(i + 1)*8 + 2] - (x[i * 8 + 2] + x[8*i + 3]*dt*sin(x[i*8 + 4] + atan(lr/(lf + lr) * tan(x[i*8 + 8])))) == 0
              #x[(i + 1)*8 + 3] - (x[i * 8 + 3] + x[8*i + 7]*dt) == 0
              x[(i + 1)*8 + 3] - (x[i * 8 + 3] + x_d*dt) == 0
              x[(i + 1)*8 + 4] - (x[i * 8 + 4] + x[8*i + 3]*dt / lr*sin(atan(lr/(lf + lr) * tan(x[i*8 + 8])))) == 0
              atan(0.5 * (lf + lr) * VehicleModel.max_long_acc / x[i*8 + 3]^2) - atan(lr/(lf + lf) * tan(x[i*8 + 8])) >= 0  #max_beta - beta
              atan(0.5 * (lf + lr) * VehicleModel.max_long_acc / x[i*8 + 3]^2) + atan(lr/(lf + lf) * tan(x[i*8 + 8])) >= 0  #max_beta + beta
         end)
    end
    return mpc_struct
end


function define_constraint_nonlinear_bycicle(mpc_struct)
    x = mpc_struct.x
    m = mpc_struct.m
    N = mpc_struct.N

    lf = VehicleModel.lf
    lr = VehicleModel.lr
    #create vehicle model constraints
    for i in 0: N-1

         #expression for tire model
         @NLexpression(m, slip_angle_f, x[i * 8 + 8] - atan((x[i * 8 + 5] + VehicleModel.lf * x[i * 8 + 6]) / x[i * 8 + 3]))
         @NLexpression(m, slip_angle_b,              - atan((x[i * 8 + 5] - VehicleModel.lf * x[i * 8 + 6]) / x[i * 8 + 3]))

         @NLexpression(m, Fbx, VehicleModel.F_long_max * x[8*i + 7]/10.0)
         @NLexpression(m, Ffy, VehicleModel.Df * slip_angle_f / VehicleModel.xmf )
         @NLexpression(m, Fby, VehicleModel.Db * slip_angle_b / VehicleModel.xmb )

         @NLconstraints(m, begin
              x[(i + 1)*8 + 1] - (x[i * 8 + 1] + dt * (x[8*i + 3]*cos(x[i*8 + 4]) - x[8*i + 5] * sin(x[8*i + 4]))) == 0
              x[(i + 1)*8 + 2] - (x[i * 8 + 2] + dt * (x[8*i + 3]*sin(x[i*8 + 4]) + x[8*i + 5] * cos(x[8*i + 4]))) == 0
              x[(i + 1)*8 + 3] - (x[i * 8 + 3] + dt * (Fbx - Ffy * sin(x[i * 8 + 8]) + VehicleModel.mass * x[i * 8 + 5] * x[i * 8 + 6]) * (1.0/VehicleModel.mass))== 0
              x[(i + 1)*8 + 4] - (x[i * 8 + 4] + dt * (x[8*i + 6])) == 0
              x[(i + 1)*8 + 5] - (x[i*8 + 5] + dt * (Fby + Ffy * cos(x[8*i + 8]) - VehicleModel.mass * x[8*i + 3] * x[8*i + 6])* (1.0/VehicleModel.mass)) == 0
              x[(i + 1)*8 + 6] - (x[i*8 + 6] + dt * (VehicleModel.lf * Ffy * cos(x[i*8 + 8]) - VehicleModel.lr * Fby)/VehicleModel.I) == 0
         end)
    end

    return mpc_struct
end


function define_constraint_start_pose(mpc_struct, startPose)
    m = mpc_struct.m
    x = mpc_struct.x
    #enforce starting point
    startPos = []
    startPosX = @constraint(m, startPosX, x[1] == startPose.x)
    startPos = vcat(startPos, startPosX)
    startPosY = @constraint(m, startPosY, x[2] == startPose.y)
    startPos = vcat(startPos, startPosY)
    startPosX_d = @constraint(m, startPosX_d, x[3] == startPose.x_d)
    startPos = vcat(startPos, startPosX_d)
    startPosPsi = @constraint(m, startPosPsi, x[4] == startPose.psi)
    startPos = vcat(startPos, startPosPsi)
    startPosY_d = @constraint(m, startPosY_d, x[5] == startPose.y_d)
    startPos = vcat(startPos, startPosY_d)
    startPosPsi_d = @constraint(m, startPosPsi_d, x[6] == startPose.psi_d)
    startPos = vcat(startPos, startPosPsi_d)
    mpc_struct.startPose = startPos
    return mpc_struct
end

function update_start_point_from_pose(mpc_struct, carPose)
     startPos = mpc_struct.startPose
     #enforce starting point
     JuMP.setRHS(startPos[1], carPose.x)
     JuMP.setRHS(startPos[2], carPose.y)
     JuMP.setRHS(startPos[3], carPose.x_d)
     JuMP.setRHS(startPos[4], carPose.psi)
     JuMP.setRHS(startPos[5], carPose.y_d)
     JuMP.setRHS(startPos[6], carPose.psi_d)
     return mpc_struct
end

function define_constraint_tangents(mpc_struct, trackPoints)
     x = mpc_struct.x
     m = mpc_struct.m
     N = mpc_struct.N
     t = @NLparameter(m, t[i=1:N*6] == trackPoints[i])
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
     mpc_struct.t = t
     return mpc_struct
end

function update_track_points(mpc_struct, trackPoints)
     #x0 = [t[i*6 + 1], t[i*6 + 2]] /midpoint
     #x1 = [t[i*6 + 3], t[i*6 + 4]] / left bound
     #x2 = [t[i*6 + 5], t[i*6 + 6]] / right bound
     m = mpc_struct.m
     t = mpc_struct.t
     N = mpc_struct.N
     for i in 0:N-1
          setvalue(t[i*6 + 1] , trackPoints[i*6 + 1])
          setvalue(t[i*6 + 2] , trackPoints[i*6 + 2])
          setvalue(t[i*6 + 3] , trackPoints[i*6 + 3])
          setvalue(t[i*6 + 4] , trackPoints[i*6 + 4])
          setvalue(t[i*6 + 5] , trackPoints[i*6 + 5])
          setvalue(t[i*6 + 6] , trackPoints[i*6 + 6])
     end
     return mpc_struct
end

#keeps the search point on the track to prevent jumping
function define_constraint_max_search_dist(mpc_struct, trackPoints)
     m = mpc_struct.m
     x = mpc_struct.x
     t = mpc_struct.t
     N = mpc_struct.N
     trackWidth = 4

     for i in 0:N-1
          p1 = [t[i*6 + 1], t[i*6 + 2]]
          @NLconstraint(m, sqrt((x[(i+1)*8 + 1] - p1[1])^2 +  (x[(i+1)*8 + 2] - p1[2])^2) <= trackWidth*1)
     end
     return mpc_struct
end


function define_objective_minimize_dist(mpc_struct)
    m = mpc_struct.m
    x = mpc_struct.x
    N = mpc_struct.N
    z = mpc_struct.z
    #z defined in init so to change objective functions without changing the mpc_struct possible
    @NLobjective(m, Min, sqrt((x[N*8+1]-z[1])^2 + (x[N*8 +2]-z[2])^2))
    return mpc_struct
end

function update_track_forward_point(mpc_struct, point)
     z = mpc_struct.z
     setvalue(z[1], point[1])
     setvalue(z[2], point[2])
     return mpc_struct
end

function define_objective_middle(mpc_struct)
    m = mpc_struct.m
    x = mpc_struct.x
    N = mpc_struct.N
    t = mpc_struct.t

    #dist(x_p, y_p, x_m, y_m) = ((x_p - x_m)^2 + (y_p - y_m)^2)^(1/2.0)
    #JuMP.register(m, :dist, 4, dist, autodiff=true)

    #@NLexpression(m, dist, sum(dist(x[(i+1)*8 + 1], x[(i+1)*8 + 1], t[i*6 + 1], t[i*6 + 2]) for i in 0:N-1))
    @NLexpression(m, dist, sum(sqrt((x[(i+1)*8 + 1] - t[i*6 + 1])^2 +  (x[(i+1)*8 + 2] - t[i*6 + 2])^2) for i in 0:5:N-1))

    @NLexpression(m, sum_speed, sum(10.0/x[i*8 + 3] for i in 1:N))
    @NLobjective(m, Min, sum_speed + dist)
    return mpc_struct
end

function define_objective(mpc_struct)
    m = mpc_struct.m
    x = mpc_struct.x
    N = mpc_struct.N
    @NLobjective(m, Min, sum(1/x[i*8 + 3] for i in 1:N))
    return mpc_struct
end

function print_mpc(mpc_struct)
    print(mpc_struct.m)
end

function solve_MPC(mpc_struct)
     m = mpc_struct.m
     status = solve(m)
     res = getvalue(mpc_struct.x)
     return res, status
end
