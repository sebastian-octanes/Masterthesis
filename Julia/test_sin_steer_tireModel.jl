
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
speed = [30]
t = 0:.05:2
dt = 0.05
controlVectorThrottle = 0*t
controlVectorSteer = sin.(2π*t) * (30.0/180.0)*pi
#controlVectorSteer = t* (30.0/180.0)*pi

dist_kin_kin =[]
dist_kin_dyn= []
dist_kin_kamsch=[]
dist_dyn_kamsch=[]

kin = []
dyn_base = []
dyn_kamsch = []

for i in speed
    println("speed", i)
    stateVectorDyn_Base= VehicleModel.CarPose(0,0,i,0, 0, 0)
    stateVectorDyn_Simple = VehicleModel.CarPose(0,0,i,0, 0, 0)
    stateVectorDyn_Lin = VehicleModel.CarPose(0,0,i,0, 0, 0)
    stateVectorDyn_Long = VehicleModel.CarPose(0,0,i,0, 0, 0)

    dyn_base = []
    dyn_simple = []
    dyn_lin = []
    dyn_long = []

    for j in range(1,1,size(t)[1])

        carControl = VehicleModel.CarControls(controlVectorThrottle[j], controlVectorSteer[j])

        stateVectorDyn_Base =  VehicleModel.dyn_model_base(stateVectorDyn_Base, carControl, dt)
        dyn_base = vcat(dyn_base, stateVectorDyn_Base)

        stateVectorDyn_Simple =  VehicleModel.dyn_model_lat_simple(stateVectorDyn_Simple, carControl, dt)
        dyn_simple = vcat(dyn_simple, stateVectorDyn_Simple)

        stateVectorDyn_Lin =  VehicleModel.dyn_model_lat_lin(stateVectorDyn_Lin, carControl, dt)
        dyn_lin = vcat(dyn_lin, stateVectorDyn_Lin)

        stateVectorDyn_Long =  VehicleModel.dyn_model_lat_lin(stateVectorDyn_Long, carControl, dt)
        dyn_long = vcat(dyn_long, stateVectorDyn_Long)
    end

    phi_base = extract_orientation(dyn_base)

    phi_simple = extract_orientation(dyn_simple)

    phi_lin = extract_orientation(dyn_lin)

    phi_long = extract_orientation(dyn_long)


    areas = 5*ones(speed)
    subplot(211)
    scatter(t, controlVectorSteer, s=areas,alpha=1.0)
    grid()

    subplot(212)

    scatter(t, phi_base, s=areas, alpha=1.0)
    #scatter(t, phi_simple, s=areas, alpha=1.0)
    #scatter(t, phi_lin, s=areas, alpha=1.0)
    scatter(t, phi_long, s=areas, alpha=1.0)

    open("outputFiles/modelDiffTires.txt", "w") do io
        writedlm(io, [t controlVectorSteer phi_base phi_simple phi_lin])
    end

end
