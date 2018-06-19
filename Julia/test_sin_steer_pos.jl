
using PyPlot
include("RaceCourse.jl")
include("VehicleModel.jl")
include("MPC.jl")
include("circular_buffer.jl")
include("smallMPC.jl")
#using CircBuffer
#using VehicleModel.CarPose, VehicleModel.CarState, VehicleModel
#using RaceCourse



#einen plot ohne beschleunigung, einen mit beschleunigung
#einen plot für positionsabweichung und einen für orientierungsabweichung


function compute_distance(kin, dyn)
    dist = 0
    for i in size(kin)
        dist_tmp = sqrt((kin[i].x - dyn[i].x)^2 + (kin[i].y - dyn[i].y)^2)
        dist = dist_tmp + dist
    end
    return dist
end
function extract_trajektory(kin)
    x = []
    y = []
    for i in range(1, 1, size(kin)[1])
        x = vcat(x, kin[i].x)
        y = vcat(y, kin[i].y)
    end
    return x,y
end

function extract_orientation(kin)
    phi = []
    for i in range(1, 1, size(kin)[1])
        phi = vcat(phi, kin[i].psi)
    end
    return phi
end

#speed = 1:1:30
speed = [7, 30]
t = 0:.05:2
dt = 0.05
controlVectorThrottle = 10*t
controlVectorSteer = sin.(2π*t) * (30.0/180.0)*pi
#controlVectorSteer = t* (30.0/180.0)*pi

dist_kin_kin =[]
dist_kin_dyn= []
dist_kin_kamsch=[]
dist_dyn_kamsch=[]

kin = []
dyn_base = []
dyn_kamsch = []

kin1 = []
dyn_base1 = []
dyn_kamsch1 = []
counter = 1

for i in speed
    stateVectorKin= VehicleModel.CarPose(0,0,i,0, 0, 0)
    stateVectorDyn_Base = VehicleModel.CarPose(0,0,i,0, 0, 0)
    stateVectorDyn_KamCircle = VehicleModel.CarPose(0,0,i,0, 0, 0)

    kin = []
    dyn_base = []
    dyn_kamsch = []


    for j in range(1,1,size(t)[1])

        carControl = VehicleModel.CarControls(controlVectorThrottle[j], controlVectorSteer[j])

        stateVectorKin =  VehicleModel.kin_bycicle_model(stateVectorKin, carControl, dt)
        kin = vcat(kin, stateVectorKin)
        #kin2[j] = vcat[kin2[j], stateVectorKin]

        stateVectorDyn_Base =  VehicleModel.dyn_model_base(stateVectorDyn_Base, carControl, dt)
        dyn_base = vcat(dyn_base, stateVectorDyn_Base)

        stateVectorDyn_KamCircle =  VehicleModel.dyn_model_kamCircle(stateVectorDyn_KamCircle, carControl, dt)
        dyn_kamsch = vcat(dyn_kamsch, stateVectorDyn_KamCircle)
    end

    dist = compute_distance(kin, dyn_base)
    dist_kin_dyn = vcat(dist_kin_dyn, dist)
    phi_kin = extract_orientation(kin)

    dist = compute_distance(kin, dyn_kamsch)
    dist_kin_kamsch = vcat(dist_kin_kamsch, dist)
    phi_dyn = extract_orientation(dyn_base)

    dist = compute_distance(dyn_base, dyn_kamsch)
    dist_dyn_kamsch = vcat(dist_dyn_kamsch, dist)
    phi_kamsch = extract_orientation(dyn_kamsch)
    if(counter == 1)
        kin1 = copy(kin)
        dyn_base1 = copy(dyn_base)
        dyn_kamsch1 = copy(dyn_kamsch)
    end
    counter = counter +1

end



    subplot(311)
    areas = 5*ones(speed)
    scatter(t, controlVectorSteer, s=areas,alpha=1.0)
    grid()

    subplot(312)
    x,y = extract_trajektory(kin)
    scatter(x, y, s=areas, alpha=1.0)
    x,y = extract_trajektory(dyn_base)
    scatter(x, y, s=areas, alpha=1.0)
    x,y = extract_trajektory(dyn_kamsch)
    scatter(x, y, s=areas, alpha=1.0)

    subplot(313)
    x,y = extract_trajektory(kin1)
    scatter(x, y, s=areas, alpha=1.0)
    x,y = extract_trajektory(dyn_base1)
    scatter(x, y, s=areas, alpha=1.0)
    x,y = extract_trajektory(dyn_kamsch1)
    scatter(x, y, s=areas, alpha=1.0)
