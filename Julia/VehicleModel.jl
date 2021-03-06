module VehicleModel

#vehicle geometry
wheelbase = 1.530 #distance between axes
lf  = wheelbase * 0.43   # distance tire to COG
lr  = wheelbase * 0.567  # distance back axtle to COG
lb  = 1.2   #width of car
Af  = 0.6    #0.6 laut chip
rad = 0.2    #radius of tires in m
mass = 230   #car + driver
I  = 1000    # kgm²
#I  = 400    # kgm²

#values for longitudinal computation
P_Engine= 53000  #40500 Watt
Cd      = 1.5    #1.5 laut chip drag coefficient
rho     = 1.225  # air desity in kg/m^3
Crr     = 0.014  #roll resistance coefficient
Cf      = 70000  #N/rad
Cb      = 70000  #N/rad
mu	    = 0.0027   #roll resistance
g       = 9.81   #earth gravity
F_long_max = 3000   # 3000N for car
F_max   = 3000 # 3000N for every wheel
F_max_back = 3274

#values to limit car_parameters for mpc
min_speed = 0.001
max_speed = 118/3.6 # 120km/h /3.6 = m/s
#max_long_acc = 7.2   #m/s**2 longitudinal acceleration max
max_long_acc = 24   #m/s**2 longitudinal acceleration max
max_throttle = 10   #used for the mpc
max_long_dec = 15   #m/s**2 longitudinal deceleration max
min_throttle = 10   #used for the mpc minus is added in the mpc
max_lat_acc = 2.0*9.81  # 2g lateral acceleration
#max_lat_acc = 4.0 *9.81  # 2g lateral acceleration
max_steering_angle = (30.0/180.0)*pi #2

#tire model
Df   = 2984.0 #N
Db   = 3274.0 #N
xmf  = 0.25 #rad
xmb  = 0.37 #rad
betaf = pi/2.0 - 0.00001#rad
betab = pi/2.0 - 0.00001#rad
yaf  = 2952.0 #N
yab  = 3270.0 #N #ya has to be smaller than D


#skidpad time 7.15
#acceleration time 4.5 (75m)


struct CarPose
    x::Float64
    y::Float64
    x_d::Float64
    psi::Float64
    y_d::Float64
    psi_d::Float64
end

struct CarControls
    throttle::Float64
    phi::Float64
end

struct CarState
    x::Float64
    y::Float64
    x_d::Float64
    psi::Float64
    y_d::Float64
    psi_d::Float64
    throttle::Float64
    phi::Float64

end


# get last state and calculate next state for it + shift the whole vector to create the new stateVector
function createNewStateVector(sV, realCarStateVector, dt, N) # sV für stateVector
    carPose = CarPose(sV[end-7], sV[end-6], sV[end-5], sV[end-4], sV[end-3], sV[end-2])
    carControl = CarControls(sV[end-1], sV[end])

    carPose = kin_bycicle_model(carPose, carControl, dt)

    sV = circshift(sV, -8)
    sV[end-2] = carPose.psi_d
    sV[end-3] = carPose.y_d
    sV[end-4] = carPose.psi
    sV[end-5] = carPose.x_d
    sV[end-6] = carPose.y
    sV[end-7] = carPose.x

    return sV
end

function computeCarStepKinModel(carPose, res,  dt)
        carControls = CarControls(res[7], res[8])
        cP = kin_bycicle_model(carPose, carControls, dt)
    return cP
end

function computeCarStepDynModelLatLin(carPose, res,  dt)
        carControls = CarControls(res[7], res[8])
        cP = dyn_model_lat_lin(carPose, carControls, dt)
    return cP
end

function computeCarStepDynModelLatSimple(carPose, res,  dt)
        carControls = CarControls(res[7], res[8])
        cP = dyn_model_lat_simple(carPose, carControls, dt)
    return cP
end

function computeCarStepDynModelBase(carPose, res,  dt)
        carControls = CarControls(res[7], res[8])
        cP = dyn_model_base(carPose, carControls, dt)
    return cP
end

function computeCarStepDynModelLong(carPose, res,  dt)
        carControls = CarControls(res[7], res[8])
        cP = dyn_model_long(carPose, carControls, dt)
    return cP
end

function computeCarStepDynKamsch(carPose, res,  dt)
        carControls = CarControls(res[7], res[8])
        cP = dyn_model_kamCircle(carPose, carControls, dt)
    return cP
end

function computeCarStepDynFinal(carPose, res,  dt)
        carControls = CarControls(res[7], res[8])
        cP = dyn_model_final(carPose, carControls, dt)
    return cP
end



function kin_bycicle_model(carPose, carControl, dt)
    beta = atan((lr/(lf + lr)) * tan(carControl.phi))
    max_beta =  atan(1.0/2 * (lf + lr) * max_lat_acc / sqrt(carPose.x_d^2 + carPose.y_d^2)^2)
    if (beta >= 0)
        beta = min(beta, max_beta)
    else
        beta = - min(-beta, max_beta)
    end
    #Fbx = VehicleModel.F_long_max * carControl.throttle/10.0
    if(carControl.throttle > 0)
        Fbx = max_long_acc * mass * carControl.throttle/10.0
    else
        Fbx = max_long_dec * mass * carControl.throttle/10.0
    end
    x = carPose.x + carPose.x_d * dt * cos(carPose.psi + beta)
    y = carPose.y + carPose.x_d * dt * sin(carPose.psi + beta)
    psi = carPose.psi + (carPose.x_d * dt/lr) * sin(beta)
    #x_d = carPose.x_d + carControl.throttle * dt
    x_d = carPose.x_d + dt * (Fbx)*(1.0/mass)

    x_d = (x_d > max_speed) ?  max_speed : x_d
    x_d = (x_d < 0.1 ) ? 0.1 : x_d
    y_d = 0
    psi_d = 0
    carPose = CarPose(x, y, x_d, psi, y_d, psi_d)
    return carPose
end

#just created to print the acc_decc line with just 7.5m/s long acc.
function kin_bycicle_model_tmp(carPose, carControl, dt)
    max_long_acc = 7.5
    beta = atan((lr/(lf + lr)) * tan(carControl.phi))
    max_beta =  atan(1.0/2 * (lf + lr) * max_lat_acc / sqrt(carPose.x_d^2 + carPose.y_d^2)^2)
    if (beta >= 0)
        beta = min(beta, max_beta)
    else
        beta = - min(-beta, max_beta)
    end
    #Fbx = VehicleModel.F_long_max * carControl.throttle/10.0
    if(carControl.throttle > 0)
        Fbx = max_long_acc * mass * carControl.throttle/10.0
    else
        Fbx = max_long_dec * mass * carControl.throttle/10.0
    end
    x = carPose.x + carPose.x_d * dt * cos(carPose.psi + beta)
    y = carPose.y + carPose.x_d * dt * sin(carPose.psi + beta)
    psi = carPose.psi + (carPose.x_d * dt/lr) * sin(beta)
    #x_d = carPose.x_d + carControl.throttle * dt
    x_d = carPose.x_d + dt * (Fbx)*(1.0/mass)

    x_d = (x_d > max_speed) ?  max_speed : x_d
    x_d = (x_d < 0.1 ) ? 0.1 : x_d
    y_d = 0
    psi_d = 0
    carPose = CarPose(x, y, x_d, psi, y_d, psi_d)
    return carPose
end


function dyn_model_lat_lin(carPose, carControl, dt)
    slip_angle_f = carControl.phi- atan((carPose.y_d + lf * carPose.psi_d)/ carPose.x_d)
    slip_angle_b =               - atan((carPose.y_d - lf * carPose.psi_d)/ carPose.x_d)

    #lat
    Ffy = 2 * pacejka_tire_model_linear(slip_angle_f, true)
    Fby = 2 * pacejka_tire_model_linear(slip_angle_b, false)

    #long
    if(carControl.throttle > 0)
        power = P_Engine * carControl.throttle/10.0
        Fbx   = power /abs(sqrt(carPose.x_d^2 + carPose.y_d^2))
        if(Fbx > 2 * VehicleModel.F_long_max)
            Fbx = 2 * VehicleModel.F_long_max
        end
        Ffx = 0
    else
        Fbx = 2 * VehicleModel.F_long_max * carControl.throttle/10.0
        Ffx = 2 * VehicleModel.F_long_max * carControl.throttle/10.0
    end

    x_new = carPose.x + dt * (carPose.x_d * cos(carPose.psi) - carPose.y_d *sin(carPose.psi))
    y_new = carPose.y + dt * (carPose.x_d * sin(carPose.psi) + carPose.y_d *cos(carPose.psi))
    xd_new = carPose.x_d + dt * (Fbx - Ffy * sin(carControl.phi) + mass * carPose.y_d * carPose.psi_d)*(1.0/mass)
    psi_new = carPose.psi + dt * carPose.psi_d
    yd_new = carPose.y_d + dt * (Fby + Ffy * cos(carControl.phi) - mass * carPose.x_d * carPose.psi_d)*(1.0 / mass)
    psid_new = carPose.psi_d + dt * (lf*Ffy*cos(carControl.phi) - lr*Fby)/I

    if(xd_new < 0.1)
        xd_new = 0.1
    end
    if(xd_new >= max_speed)
        xd_new = max_speed -0.001
    end
    carPoseNew = CarPose(x_new, y_new, xd_new, psi_new, yd_new, psid_new)
    return carPoseNew
end


function dyn_model_lat_simple(carPose, carControl, dt)
    slip_angle_f = carControl.phi- atan((carPose.y_d + lf * carPose.psi_d)/ carPose.x_d)
    slip_angle_b =               - atan((carPose.y_d - lf * carPose.psi_d)/ carPose.x_d)

    #lat
    Ffy = 2 * pacejka_tire_model(slip_angle_f, true)
    Fby = 2 * pacejka_tire_model(slip_angle_b, false)

    #long
    if(carControl.throttle > 0)
        power = P_Engine * carControl.throttle/10.0
        Fbx   = power /abs(sqrt(carPose.x_d^2 + carPose.y_d^2))
        if(Fbx > 2 * VehicleModel.F_long_max)
            Fbx = 2 * VehicleModel.F_long_max
        end
        Ffx = 0
    else
        Fbx = 2 * VehicleModel.F_long_max * carControl.throttle/10.0
        Ffx = 2 * VehicleModel.F_long_max * carControl.throttle/10.0
    end

    x_new = carPose.x + dt * (carPose.x_d * cos(carPose.psi) - carPose.y_d *sin(carPose.psi))
    y_new = carPose.y + dt * (carPose.x_d * sin(carPose.psi) + carPose.y_d *cos(carPose.psi))
    xd_new = carPose.x_d + dt * (Fbx - Ffy * sin(carControl.phi) + mass * carPose.y_d * carPose.psi_d)*(1.0/mass)
    psi_new = carPose.psi + dt * carPose.psi_d
    yd_new = carPose.y_d + dt * (Fby + Ffy * cos(carControl.phi) - mass * carPose.x_d * carPose.psi_d)*(1.0 / mass)
    psid_new = carPose.psi_d + dt * (lf*Ffy*cos(carControl.phi) - lr*Fby)/I

    if(xd_new < 0.1)
        xd_new = 0.1
    end
    if(xd_new >= max_speed)
        xd_new = max_speed -0.001
    end
    carPoseNew = CarPose(x_new, y_new, xd_new, psi_new, yd_new, psid_new)
    return carPoseNew
end



function dyn_model_base(carPose, carControl, dt)
    slip_angle_f = carControl.phi- atan((carPose.y_d + lf * carPose.psi_d)/ carPose.x_d)
    slip_angle_b =               - atan((carPose.y_d - lf * carPose.psi_d)/ carPose.x_d)

    #lat
    Ffy = 2 * pacejka_tire_model_complex(slip_angle_f, true)
    Fby = 2 * pacejka_tire_model_complex(slip_angle_b, false)

    #long
    Fbx = 0
    Ffx = 0
    if(carControl.throttle > 0)
        power = P_Engine * carControl.throttle/10.0
        Fbx   = power /abs(sqrt(carPose.x_d^2 + carPose.y_d^2))
        if(Fbx > 2 * VehicleModel.F_long_max)
            Fbx = 2 * VehicleModel.F_long_max
        end
        Ffx = 0
    else
        Fbx = 2 * VehicleModel.F_long_max * carControl.throttle/10.0
        Ffx = 2 * VehicleModel.F_long_max * carControl.throttle/10.0
    end

    x_new = carPose.x + dt * (carPose.x_d * cos(carPose.psi) - carPose.y_d *sin(carPose.psi))
    y_new = carPose.y + dt * (carPose.x_d * sin(carPose.psi) + carPose.y_d *cos(carPose.psi))
    xd_new = carPose.x_d + dt * (Fbx - Ffy * sin(carControl.phi) + mass * carPose.y_d * carPose.psi_d)*(1.0/mass)
    psi_new = carPose.psi + dt * carPose.psi_d
    yd_new = carPose.y_d + dt * (Fby + Ffy * cos(carControl.phi) - mass * carPose.x_d * carPose.psi_d)*(1.0 / mass)
    psid_new = carPose.psi_d + dt * (lf*Ffy*cos(carControl.phi) - lr*Fby)/I

    if(xd_new < 0.1)
        xd_new = 0.1
    end
    if(xd_new >= max_speed)
        xd_new = max_speed -0.001
    end
    carPoseNew = CarPose(x_new, y_new, xd_new, psi_new, yd_new, psid_new)
    return carPoseNew
end


function dyn_model_kamCircle(carPose, carControl, dt)
    slip_angle_f = carControl.phi- atan((carPose.y_d + lf * carPose.psi_d)/ carPose.x_d)
    slip_angle_b =               - atan((carPose.y_d - lf * carPose.psi_d)/ carPose.x_d)

    #lat
    Ffy = 2 * pacejka_tire_model_complex(slip_angle_f, true)
    Fby = 2 * pacejka_tire_model_complex(slip_angle_b, false)

    #long
    Fbx = 0
    Ffx = 0
    if(carControl.throttle > 0)
        power = P_Engine * carControl.throttle/10.0
        Fbx   = power /abs(sqrt(carPose.x_d^2 + carPose.y_d^2))
        if(Fbx > 2 * VehicleModel.F_long_max)
            Fbx = 2 * VehicleModel.F_long_max
        end
        Ffx = 0
    else
        Fbx = 2 * VehicleModel.F_long_max * carControl.throttle/10.0
        Ffx = 2 * VehicleModel.F_long_max * carControl.throttle/10.0
    end


    #Kammscher Kreis
    Ff = sqrt(Ffy^2 + Ffx^2)
    if(Ff > 2* VehicleModel.F_long_max)
        alpha = atan2(Ffx, Ffy)
        Ffx = sin(alpha) * VehicleModel.F_long_max * 2
        Ffy = cos(alpha) * VehicleModel.F_long_max * 2
    end

    Fb = sqrt(Fbx^2 + Fby^2)
    if(Fb > 2 * VehicleModel.F_max_back)
        alpha = atan2(Fbx, Fby)
        Fbx = sin(alpha) * VehicleModel.F_long_max * 2
        Fby = cos(alpha) * VehicleModel.F_long_max * 2
    end


    #base model from here again
    x_new = carPose.x + dt * (carPose.x_d * cos(carPose.psi) - carPose.y_d *sin(carPose.psi))
    y_new = carPose.y + dt * (carPose.x_d * sin(carPose.psi) + carPose.y_d *cos(carPose.psi))
    xd_new = carPose.x_d + dt * (Fbx - Ffy * sin(carControl.phi) + mass * carPose.y_d * carPose.psi_d)*(1.0/mass)
    psi_new = carPose.psi + dt * carPose.psi_d
    yd_new = carPose.y_d + dt * (Fby + Ffy * cos(carControl.phi) - mass * carPose.x_d * carPose.psi_d)*(1.0 / mass)
    psid_new = carPose.psi_d + dt * (lf*Ffy*cos(carControl.phi) - lr*Fby)/I

    if(xd_new < 0.1)
        xd_new = 0.1
    end
    if(xd_new >= max_speed)
        xd_new = max_speed -0.001
    end
    carPoseNew = CarPose(x_new, y_new, xd_new, psi_new, yd_new, psid_new)
    return carPoseNew
end


#this model enhances the standard "enhanced_long" model by incorporating the friction and air resistance

function dyn_model_long(carPose, carControl, dt)
    slip_angle_f = carControl.phi- atan((carPose.y_d + lf * carPose.psi_d)/ carPose.x_d)
    slip_angle_b =               - atan((carPose.y_d - lf * carPose.psi_d)/ carPose.x_d)

    #lat
    Ffy =  2.0* pacejka_tire_model_complex(slip_angle_f, true)
    Fby =  2.0* pacejka_tire_model_complex(slip_angle_b, false)


    #this part is enhanced compared to enhanced_long
    Faero = 1/2.0 * rho * Cd * Af * carPose.x_d^2
	Fzf   = mass*g*lf / (lf + lr)
	Fzr   = mass*g*lr / (lf + lr)
	Rxf   = mu * Fzf
	Rxr   = mu * Fzr
    println("Faero", Faero)
    println("Freib", Rxf + Rxr)

    #as car can only accelerate forward or break in simulation that decreases complexity for the if case
    #long
    if(carControl.throttle > 0)
        power = P_Engine * carControl.throttle/10.0
        Fbx   = power /abs(sqrt(carPose.x_d^2 + carPose.y_d^2))
        if(Fbx > 2 * VehicleModel.F_long_max)
            Fbx = 2 * VehicleModel.F_long_max
        end
        Ffx = 0
    else
        Fbx = 2 * VehicleModel.F_long_max * carControl.throttle/10.0
        Ffx = 2 * VehicleModel.F_long_max * carControl.throttle/10.0
    end

    x_new = carPose.x + dt * (carPose.x_d * cos(carPose.psi) - carPose.y_d *sin(carPose.psi))
    y_new = carPose.y + dt * (carPose.x_d * sin(carPose.psi) + carPose.y_d *cos(carPose.psi))
    #this part is enhanced compared to enhanced_long
    xd_new = carPose.x_d + dt * (Fbx -Rxf -Rxr -Faero -Ffy * sin(carControl.phi) + mass * carPose.y_d * carPose.psi_d)*(1.0/mass)
    #base from here
    psi_new = carPose.psi + dt * carPose.psi_d
    yd_new = carPose.y_d + dt * (Fby + Ffy * cos(carControl.phi) - mass * carPose.x_d * carPose.psi_d)*(1.0 / mass)
    psid_new = carPose.psi_d + dt * (lf*Ffy*cos(carControl.phi) - lr*Fby)/I

    if(xd_new < 0.1)
        xd_new = 0.1
    end
    if(xd_new >= max_speed)
        xd_new = max_speed - 0.001
    end
    carPoseNew = CarPose(x_new, y_new, xd_new, psi_new, yd_new, psid_new)
    return carPoseNew
end

function dyn_model_final(carPose, carControl, dt)
    slip_angle_f = carControl.phi- atan((carPose.y_d + lf * carPose.psi_d)/ carPose.x_d)
    slip_angle_b =               - atan((carPose.y_d - lf * carPose.psi_d)/ carPose.x_d)

    #lat
    Ffy = 2 * pacejka_tire_model_complex(slip_angle_f, true)
    Fby = 2 * pacejka_tire_model_complex(slip_angle_b, false)

    #long
    Fbx = 0
    Ffx = 0
    if(carControl.throttle > 0)
        power = P_Engine * carControl.throttle/10.0
        Fbx   = power /abs(sqrt(carPose.x_d^2 + carPose.y_d^2))
        if(Fbx > 2 * VehicleModel.F_long_max)
            Fbx = 2 * VehicleModel.F_long_max
        end
        Ffx = 0
    else
        Fbx = 2 * VehicleModel.F_long_max * carControl.throttle/10.0
        Ffx = 2 * VehicleModel.F_long_max * carControl.throttle/10.0
    end
    #this part is enhanced compared to enhanced_long
    Faero = 1/2.0 * rho * Cd * Af * carPose.x_d^2
	Fzf   = mass*g*lf / (lf + lr)
	Fzr   = mass*g*lr / (lf + lr)
	Rxf   = mu * Fzf
	Rxr   = mu * Fzr
    #as car can only accelerate forward or break in simulation that decreases complexity for the if case


    #Kammscher Kreis
    Ff = sqrt(Ffy^2 + Ffx^2)
    if(Ff > 2* VehicleModel.F_long_max)
        alpha = atan2(Ffx, Ffy)
        Ffx = sin(alpha) * VehicleModel.F_long_max * 2
        Ffy = cos(alpha) * VehicleModel.F_long_max * 2
    end

    Fb = sqrt(Fbx^2 + Fby^2)
    if(Fb > 2 * VehicleModel.F_max_back)
        alpha = atan2(Fbx, Fby)
        Fbx = sin(alpha) * VehicleModel.F_long_max * 2
        Fby = cos(alpha) * VehicleModel.F_long_max * 2
    end


    #base model from here again
    x_new = carPose.x + dt * (carPose.x_d * cos(carPose.psi) - carPose.y_d *sin(carPose.psi))
    y_new = carPose.y + dt * (carPose.x_d * sin(carPose.psi) + carPose.y_d *cos(carPose.psi))
    xd_new = carPose.x_d + dt * (Fbx -Rxf -Rxr -Faero -Ffy * sin(carControl.phi) + mass * carPose.y_d * carPose.psi_d)*(1.0/mass)
    psi_new = carPose.psi + dt * carPose.psi_d
    yd_new = carPose.y_d + dt * (Fby + Ffy * cos(carControl.phi) - mass * carPose.x_d * carPose.psi_d)*(1.0 / mass)
    psid_new = carPose.psi_d + dt * (lf*Ffy*cos(carControl.phi) - lr*Fby)/I

    if(xd_new < 0.1)
        xd_new = 0.1
    end
    if(xd_new >= max_speed)
        xd_new = max_speed -0.001
    end
    carPoseNew = CarPose(x_new, y_new, xd_new, psi_new, yd_new, psid_new)
    return carPoseNew
end



function pacejka_tire_model_linear(slip_angle, front)
    #linear model
    if(front == true)
        D = Df
        xm = xmf
        C = Cf
    else
        D = Db
        xm = xmb
        C = Cb
    end
    #C = D/xm
    y = C * slip_angle
    if(y > 3000)
        y= 3000
    elseif(y < -3000)
        y = -3000
    end
    return y
end

function pacejka_tire_model(slip_angle, front)
    #dont forget that CBE have to be computed just once and then not again
    if(front == true)
        D = Df
        ya = yaf
        beta = betaf
    else
        D = Db
        ya = yab
        beta = betab
    end
    C = 1 + (1 - (2.0/pi))* asin(ya/D)
    B = tan(beta)/(C*D)
    y = D*sin(C*atan(B*slip_angle))
    return y
end


 function pacejka_tire_model_complex(slip_angle, front)
    #very good approximation of our vehicle
    if(front == true)
        D = Df
        ya = yaf
        beta = betaf
        xm = xmf
    else
        D = Db
        ya = yab
        beta = betab
        xm = xmb
    end
    C = 1 + (1 - (2.0/pi))* asin(ya/D)
    B = tan(beta)/(C*D)
    E = (B * xm - tan(pi/(2.0*C)))/(B*xm - atan(B*xm))
    y = D*sin(C*atan(B*slip_angle - E*(B*slip_angle -atan(B*slip_angle))))
    return y
end


export CarState, computeRealCarStep, simulateRealCarForNextStep, computeTimeStep, CarPose, CarControls, max_steering_angle, max_long_acc, max_long_dec, createNewStateVector

end
