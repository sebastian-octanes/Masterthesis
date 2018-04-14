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
    #z     #search radius constraint prevents from jumping
end


function init_MPC(mpc_struct, N_, dt, startPose, printLevel)
     m = Model(solver = IpoptSolver(print_level = printLevel, max_iter=5000))
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
         @NLconstraints(m, begin
              x[(i + 1)*8 + 1] - (x[i * 8 + 1] + x[8*i + 3]*dt*cos(x[i*8 + 4] + atan(lr/(lf + lr) * tan(x[i*8 + 8])))) == 0
              x[(i + 1)*8 + 2] - (x[i * 8 + 2] + x[8*i + 3]*dt*sin(x[i*8 + 4] + atan(lr/(lf + lr) * tan(x[i*8 + 8])))) == 0
              x[(i + 1)*8 + 3] - (x[i * 8 + 3] + x[8*i + 7]*dt) == 0
              x[(i + 1)*8 + 4] - (x[i * 8 + 4] + x[8*i + 3]*dt / lr*sin(atan(lr/(lf + lr) * tan(x[i*8 + 8])))) == 0
              atan(0.5 * (lf + lr) * VehicleModel.max_long_acc / x[i*8 + 3]^2) - atan(lr/(lf + lf) * tan(x[i*8 + 8])) >= 0  #max_beta - beta
              atan(0.5 * (lf + lr) * VehicleModel.max_long_acc / x[i*8 + 3]^2) + atan(lr/(lf + lf) * tan(x[i*8 + 8])) >= 0  #max_beta + beta
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
#=
     midTrackPoints = []
     for i in 1:(N-1)
          midTrackPoints = vcat(midTrackPoints, trackPoints[i*6 +1])
          midTrackPoints = vcat(midTrackPoints, trackPoints[i*6 +2])
     end
     z = @NLparameter(m, z[i = 1:N*2] == midTrackPoints[i])
=#
     for i in 0:N-1
          p1 = [t[i*6 + 1], t[i*6 + 2]]
          #p1 = [z[i*2 + 1], z[i*2 + 2]]
          @NLconstraint(m, sqrt((x[(i+1)*8 + 1] - p1[1])^2 +  (x[(i+1)*8 + 2] - p1[2])^2) <= trackWidth*1)
     end
     return mpc_struct
end




function define_objective(mpc_struct)
    m = mpc_struct.m
    x = mpc_struct.x
    N = mpc_struct.N
    @NLobjective(m, Max, sum(x[i*8 + 3] for i in 1:N))
    return mpc_struct
end

function print_mpc(mpc_struct)
    print(mpc_struct.m)
end

function solve_MPC(mpc_struct)
     m = mpc_struct.m
     solve(m)
     res = getvalue(mpc_struct.x)
     return res
end
