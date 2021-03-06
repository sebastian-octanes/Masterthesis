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
    beta_max
end


function init_MPC(mpc_struct, N_, dt, startPose, printLevel, max_speed, track_width)
     #m = Model(solver = IpoptSolver(tol=1e-1, print_level = printLevel, max_iter= 300))
     m = Model(solver = IpoptSolver(tol=1e-1, print_level = printLevel))
     N = N_

     lbx = []
     ubx = []
     start = []

     lbx_ = [-Inf, -Inf, VehicleModel.min_speed, -Inf, -Inf, -Inf,  -VehicleModel.min_throttle, -VehicleModel.max_steering_angle]
     ubx_ = [ Inf,  Inf, max_speed,               Inf,  Inf,  Inf,   VehicleModel.max_throttle,  VehicleModel.max_steering_angle]
     start_=[startPose.x, startPose.y, startPose.x_d, startPose.psi, startPose.y_d, startPose.psi_d, 0, 0]
     for i in 0:N
          lbx = vcat(lbx, lbx_) #add lower bounds to vector
          ubx = vcat(ubx, ubx_) #add upper bounds to vector
          start = vcat(start, start_) #add initial guess to vector
     end

     x = @variable(m, lbx[i] <= x[i = 1:(N+1)*8] <= ubx[i], start = start[i]) #set bounds and initial guess and create x-vector
     #defined in init to make change of objective functions possible without changing mpc_struct
     #call setforwardPoint before using the objective function
     forwardDummy =[0,0,0,0,0,0]
     z = @NLparameter(m, z[i=1:6] == forwardDummy[i])
     mpc_struct.z = z
     trackPoints= ones(N*6)
     #define t here to be able to use softconstraints later
     t = @NLparameter(m, t[i=1:N*6] == trackPoints[i])
     mpc_struct.t = t

     mpc_struct.m = m
     mpc_struct.x = x
     mpc_struct.N = N
     mpc_struct.beta_max = VehicleModel.max_lat_acc
    return mpc_struct
end

function init_MPC(mpc_struct, N_, dt, startPose, printLevel, max_speed, track_width, beta)
     "init_MPC_beta"
     m = Model(solver = IpoptSolver(tol=1e-1, print_level = printLevel, max_iter= 500))
     N = N_

     lbx = []
     ubx = []
     start = []

     lbx_ = [-Inf, -Inf, VehicleModel.min_speed, -Inf, -Inf, -Inf,  -VehicleModel.min_throttle, -VehicleModel.max_steering_angle]
     ubx_ = [ Inf,  Inf, max_speed,               Inf,  Inf,  Inf,   VehicleModel.max_throttle,  VehicleModel.max_steering_angle]
     start_=[startPose.x, startPose.y, startPose.x_d, startPose.psi, startPose.y_d, startPose.psi_d, 0, 0]
     for i in 0:N
          lbx = vcat(lbx, lbx_) #add lower bounds to vector
          ubx = vcat(ubx, ubx_) #add upper bounds to vector
          start = vcat(start, start_) #add initial guess to vector
     end

     x = @variable(m, lbx[i] <= x[i = 1:(N+1)*8] <= ubx[i], start = start[i]) #set bounds and initial guess and create x-vector
     #defined in init to make change of objective functions possible without changing mpc_struct
     #call setforwardPoint before using the objective function
     forwardDummy =[0,0,0,0,0,0]
     z = @NLparameter(m, z[i=1:6] == forwardDummy[i])
     mpc_struct.z = z
     trackPoints= ones(N*6)
     #define t here to be able to use softconstraints later
     t = @NLparameter(m, t[i=1:N*6] == trackPoints[i])
     mpc_struct.t = t

     mpc_struct.m = m
     mpc_struct.x = x
     mpc_struct.N = N
     mpc_struct.beta_max = beta
    return mpc_struct
end


function define_constraint_kin_bycicle(mpc_struct)
    x = mpc_struct.x
    m = mpc_struct.m
    N = mpc_struct.N

    lf = VehicleModel.lf
    lr = VehicleModel.lr
    #create vehicle model constraints
    for i in 0: N-1
         #@NLexpression(m, x_dd, (VehicleModel.F_long_max * x[8*i + 7]/10.0)* 1.0/VehicleModel.mass)
         @NLexpression(m, x_dd, (VehicleModel.max_long_acc * x[8*i + 7]/10.0))

         @NLconstraints(m, begin
              x[(i + 1)*8 + 1] - (x[i * 8 + 1] + x[8*i + 3]*dt*cos(x[i*8 + 4] + atan(lr/(lf + lr) * tan(x[i*8 + 8])))) == 0
              x[(i + 1)*8 + 2] - (x[i * 8 + 2] + x[8*i + 3]*dt*sin(x[i*8 + 4] + atan(lr/(lf + lr) * tan(x[i*8 + 8])))) == 0
              #x[(i + 1)*8 + 3] - (x[i * 8 + 3] + x[8*i + 7]*dt) == 0
              x[(i + 1)*8 + 3] - (x[i * 8 + 3] + x_dd*dt) == 0
              x[(i + 1)*8 + 4] - (x[i * 8 + 4] + x[8*i + 3]*dt / lr*sin(atan(lr/(lf + lr) * tan(x[i*8 + 8])))) == 0
              atan(0.5 * (lf + lr) * mpc_struct.beta_max / x[i*8 + 3]^2) - atan(lr/(lf + lf) * tan(x[i*8 + 8])) >= 0  #max_beta - beta
              atan(0.5 * (lf + lr) * mpc_struct.beta_max / x[i*8 + 3]^2) + atan(lr/(lf + lf) * tan(x[i*8 + 8])) >= 0  #max_beta + beta
         end)
    end
    return mpc_struct
end


function define_constraint_dyn_bycicle(mpc_struct)
    x = mpc_struct.x
    m = mpc_struct.m
    N = mpc_struct.N

    #create vehicle model constraints
    for i in 0: N-1

         #expression for tire model
         @NLexpression(m, slip_angle_f, x[i * 8 + 8] - atan((x[i * 8 + 5] + VehicleModel.lf * x[i * 8 + 6]) / x[i * 8 + 3]))
         @NLexpression(m, slip_angle_b,              - atan((x[i * 8 + 5] - VehicleModel.lf * x[i * 8 + 6]) / x[i * 8 + 3]))

         @NLexpression(m, Fbx, VehicleModel.F_long_max * x[8*i + 7]/10.0)
         @NLexpression(m, Ffy, VehicleModel.Cf * slip_angle_f)
         @NLexpression(m, Fby, VehicleModel.Cb * slip_angle_b)

         @NLconstraints(m, begin
              x[(i + 1)*8 + 1] - (x[i * 8 + 1] + dt * (x[8*i + 3]*cos(x[i*8 + 4]) - x[8*i + 5] * sin(x[8*i + 4]))) == 0
              x[(i + 1)*8 + 2] - (x[i * 8 + 2] + dt * (x[8*i + 3]*sin(x[i*8 + 4]) + x[8*i + 5] * cos(x[8*i + 4]))) == 0
              x[(i + 1)*8 + 3] - (x[i * 8 + 3] + dt * (Fbx - Ffy * sin(x[i * 8 + 8]) + VehicleModel.mass * x[i * 8 + 5] * x[i * 8 + 6]) * (1.0/VehicleModel.mass))== 0
              x[(i + 1)*8 + 4] - (x[i * 8 + 4] + dt * (x[8*i + 6])) == 0
              x[(i + 1)*8 + 5] - (x[i * 8 + 5] + dt * (Fby + Ffy * cos(x[8*i + 8]) - VehicleModel.mass * x[8*i + 3] * x[8*i + 6])* (1.0/VehicleModel.mass)) == 0
              x[(i + 1)*8 + 6] - (x[i * 8 + 6] + dt * (VehicleModel.lf * Ffy * cos(x[i*8 + 8]) - VehicleModel.lr * Fby)/VehicleModel.I) == 0
              #Fby + 3000 >= 0
              #Fby - 3000 <= 0
              #Ffy + 3000 >= 0
              #Ffy - 3000 <= 0

         end)
    end

    return mpc_struct
end

#enforces the vehicle position in the first time step t0
#without the mpc can move the x,y,v values to whatever point that min/maximizes the cost function
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

#updates the new position of the car in the mpc
#has to be called after every optimization step!
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

#tangential constraint that keeps the vehicle on the track, was later replaced with a vector projection
#didn't increase the performance but is more flexible in the long run
function define_constraint_tangents(mpc_struct, trackPoints)
     x = mpc_struct.x
     m = mpc_struct.m
     N = mpc_struct.N
     #t = @NLparameter(m, t[i=1:N*6] == trackPoints[i])
     t = mpc_struct.t

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


#this functions updates the midpoint, left and right bounds for every prediction step of the mpc
#is needed for the hard and softconstraints
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


#maximes the speed in every prediction point. Very good performing but more or less useless
function define_objective_max_speed(mpc_struct)
    m = mpc_struct.m
    x = mpc_struct.x
    N = mpc_struct.N
    @NLobjective(m, Min, sum(1/x[i*8 + 3] for i in 1:N))
    return mpc_struct
end

#minimzes the distance from the last prediction point to the forward point that is updatet after every optimization step
#this constraint can be used without the tangential constraint but gets very unstable with speed higher than 3-4m/s even with the same vehicle model as in the simulation environment
function define_objective_minimize_dist(mpc_struct)
    m = mpc_struct.m
    x = mpc_struct.x
    N = mpc_struct.N
    z = mpc_struct.z
    #z defined in init to make changes to objective functions without changing the mpc_struct possible
    #@NLobjective(m, Min, sqrt((x[(N-1)*8 + 1]-0)^2 + (x[(N-1)*8 + 2]-10)^2))
    @NLobjective(m, Min, sqrt((x[(N-1)*8 + 1]- z[1])^2 + (x[(N-1)*8 + 2]-z[2])^2))

    return mpc_struct
end

#maximizes the driven distance on the track by taking the minimize dist function computing the distance with a vector projection
function define_objective_max_track_dist(mpc_struct)
    m = mpc_struct.m
    x = mpc_struct.x
    N = mpc_struct.N
    z = mpc_struct.z


    #a2 = a - ((a*b)/(b*b))*b
    #c = (ax * bx + ay *by)/(bx^2 + by^2)
    #a2x = ax - c * bx
    #a2y = ay - c * by
    #dist = sqrt((a2x)^2 + (a2y)^2)

    dist(ax, ay, bx, by) =  sqrt((ax - (ax * bx + ay *by)/(bx^2 + by^2)*bx)^2 + (ay - (ax * bx + ay *by)/(bx^2 + by^2)*by)^2)
    JuMP.register(m, :dist, 4, dist, autodiff=true)
    @NLexpression(m, min_dist , dist(x[N*8 + 1] - z[1], x[N*8 + 2] - z[2], z[3]- z[1], z[4] - z[2]))
    @NLobjective(m, Min, min_dist)
    return mpc_struct
end

#adds a softconstraint to the minimize dist function that tries to keep the distance to the middle of the track minimal
#doesn't have a good performance but functions
function define_objective_minimize_dist_soft_const_lin(mpc_struct, a = 6, b = 1)
    m = mpc_struct.m
    x = mpc_struct.x
    N = mpc_struct.N
    z = mpc_struct.z
    t = mpc_struct.t
    #z defined in init so to change objective functions without changing the mpc_struct possible
    #@NLobjective(m, Min, sqrt((x[(N-1)*8 + 1]-0)^2 + (x[(N-1)*8 + 2]-10)^2))
    @NLexpression(m, min_dist, sqrt((x[(N-1)*8 + 1]- z[1])^2 + (x[(N-1)*8 + 2]-z[2])^2))

    dist(xX, xY, x0X, x0Y, x1X, x1Y) =  abs((xX - x0X)*(x1X - x0X) + (xY - x0Y)*(x1Y - x0Y)) / sqrt((x1X - x0X)^2 + (x1Y - x0Y)^2)
    JuMP.register(m, :dist, 6, dist, autodiff=true)
    @NLexpression(m, soft_constraint, sum(dist(x[(i+1)*8 + 1], x[(i+1)*8 + 2], t[i*6 + 1], t[i*6 + 2], t[i*6 + 3], t[i*6 + 4]) for i in 2:5:N))

    #set a and b according how strongly you want to follow you point ahead of you and how important it is to stay in the middle of the road
    #at a=6 it pretty much hits the track boundary
    @NLobjective(m, Min, a* min_dist + b*soft_constraint)
    return mpc_struct
end

function define_objective_minimize_dist_soft_const_quad(mpc_struct, a = 6, b = 1)
    m = mpc_struct.m
    x = mpc_struct.x
    N = mpc_struct.N
    z = mpc_struct.z
    t = mpc_struct.t
    #z defined in init so to change objective functions without changing the mpc_struct possible
    #@NLobjective(m, Min, sqrt((x[(N-1)*8 + 1]-0)^2 + (x[(N-1)*8 + 2]-10)^2))
    @NLexpression(m, min_dist, sqrt((x[(N-1)*8 + 1]- z[1])^2 + (x[(N-1)*8 + 2]-z[2])^2))

    alpha = 20

    dist(xX, xY, x0X, x0Y, x1X, x1Y) =  abs((xX - x0X)*(x1X - x0X) + (xY - x0Y)*(x1Y - x0Y)) / sqrt((x1X - x0X)^2 + (x1Y - x0Y)^2)
    JuMP.register(m, :dist, 6, dist, autodiff=true)
    @NLexpression(m, soft_constraint, sum(alpha * dist(x[(i+1)*8 + 1], x[(i+1)*8 + 2], t[i*6 + 1], t[i*6 + 2], t[i*6 + 3], t[i*6 + 4])^2 for i in 2:5:N))

    #set a and b according how strongly you want to follow you point ahead of you and how important it is to stay in the middle of the road
    #at a=6 it pretty much hits the track boundary
    @NLobjective(m, Min, a* min_dist + b*soft_constraint)
    return mpc_struct
end


#changes the softconstraint to a softconstraint that has low cost as long as car stays on the track
#for now it is not working very good
function define_objective_minimize_dist_soft_const_ext(mpc_struct, a, b)
    m = mpc_struct.m
    x = mpc_struct.x
    N = mpc_struct.N
    z = mpc_struct.z
    t = mpc_struct.t
    #z defined in init so to change objective functions without changing the mpc_struct possible
    #@NLobjective(m, Min, sqrt((x[(N-1)*8 + 1]-0)^2 + (x[(N-1)*8 + 2]-10)^2))
    @NLexpression(m, min_dist, sqrt((x[(N-1)*8 + 1]- z[1])^2 + (x[(N-1)*8 + 2]-z[2])^2))


    alpha = 6
    k1 = - trackWidth/2.0
    k2 =   trackWidth/2.0
    cost(xX, xY, x0X, x0Y, x1X, x1Y) = exp(alpha*(k1 + dist(xX, xY, x0X, x0Y, x1X, x1Y)))

    dist(xX, xY, x0X, x0Y, x1X, x1Y) =  abs((xX - x0X)*(x1X - x0X) + (xY - x0Y)*(x1Y - x0Y)) / sqrt((x1X - x0X)^2 + (x1Y - x0Y)^2)
    JuMP.register(m, :dist, 6, dist, autodiff=true)
    JuMP.register(m, :cost, 6, cost, autodiff=true)
    @NLexpression(m, soft_constraint, sum(cost(x[(i+1)*8 + 1], x[(i+1)*8 + 2], t[i*6 + 1], t[i*6 + 2], t[i*6 + 3], t[i*6 + 4]) for i in 2:2:N-1))

    #a= 10
    #b = 1
    @NLobjective(m, Min, a*min_dist + b*soft_constraint)
    return mpc_struct
end

function define_objective_minimize_dist_soft_const_alpha(mpc_struct, a, b)
    m = mpc_struct.m
    x = mpc_struct.x
    N = mpc_struct.N
    z = mpc_struct.z
    t = mpc_struct.t
    #z defined in init so to change objective functions without changing the mpc_struct possible
    #@NLobjective(m, Min, sqrt((x[(N-1)*8 + 1]-0)^2 + (x[(N-1)*8 + 2]-10)^2))
    @NLexpression(m, min_dist, sqrt((x[(N-1)*8 + 1]- z[1])^2 + (x[(N-1)*8 + 2]-z[2])^2))



    k1 =  trackWidth/2.0
    k2 =  -trackWidth/2.0
    alpha2 = 0.99
    cost(xX, xY, x0X, x0Y, x1X, x1Y) = abs(alpha2/(k1 - dist(xX, xY, x0X, x0Y, x1X, x1Y)) + alpha2/(k2 - dist(xX, xY, x0X, x0Y, x1X, x1Y)))

    dist(xX, xY, x0X, x0Y, x1X, x1Y) =  abs((xX - x0X)*(x1X - x0X) + (xY - x0Y)*(x1Y - x0Y)) / sqrt((x1X - x0X)^2 + (x1Y - x0Y)^2)
    JuMP.register(m, :dist, 6, dist, autodiff=true)
    JuMP.register(m, :cost, 6, cost, autodiff=true)
    @NLexpression(m, soft_constraint, sum(cost(x[(i+1)*8 + 1], x[(i+1)*8 + 2], t[i*6 + 1], t[i*6 + 2], t[i*6 + 3], t[i*6 + 4]) for i in 2:2:N-1))

    #a= 10
    #b = 1
    @NLobjective(m, Min, a*min_dist + b*soft_constraint)
    return mpc_struct
end


#this function is used to update the point the minimize_dist cost function minimizes the distance to
function update_track_forward_point(mpc_struct, point)
     z = mpc_struct.z
     setvalue(z[1], point[1])
     setvalue(z[2], point[2])
     return mpc_struct
end

#this function is used to update the point the minimize_dist cost function minimizes the distance to
function update_track_forward_point_bounds(mpc_struct, point, track_points)
     z = mpc_struct.z
     #forward point
     setvalue(z[1], point[1])
     setvalue(z[2], point[2])
     #corresponding left point
     setvalue(z[3], track_points[1])
     setvalue(z[4], track_points[2])
     #corresponding right point
     setvalue(z[5], track_points[3])
     setvalue(z[6], track_points[4])

     return mpc_struct
end

#prints the mpc configuration (who would have though...)
function print_mpc(mpc_struct)
    print(mpc_struct.m)
end

#here happens all the magic
function solve_MPC(mpc_struct)
     m = mpc_struct.m
     status = solve(m)
     res = getvalue(mpc_struct.x)
     return res, status
end
