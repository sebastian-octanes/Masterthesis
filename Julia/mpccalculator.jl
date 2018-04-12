include("MPC.jl")
include("VehicleModel.jl")
include("RaceCourse.jl")
#using VehicleModel
#using MPC
#using RaceCourse
using PyPlot

N = 100
dt = 0.1
startPose = VehicleModel.CarPose(0,0,0.01,pi/2,0 ,0 )
stateVector = []
itpTrack, itpOutBound, itpInBound = RaceCourse.buildRaceTrack(15, 4, 15, 0)
start_=[startPose.x, startPose.y, startPose.x_d, startPose.psi, 0, 0, 0, 0]
for i in 0:N
    stateVector = vcat(stateVector, start_) #add initial guess to vector
end

evalPoints = RaceCourse.getSplinePositions(itpTrack, stateVector, N)
tangentPoints = RaceCourse.computeGradientPoints_(itpOutBound, itpInBound, evalPoints, N)
m = MPC.initMPC(N, dt, startPose, tangentPoints,3)
#print stuff
prin_x = []
prin_y = []

for i in 1:20
    res = MPC.solveMPC()
    push!(prin_x, res[1])
    push!(prin_y, res[2])
    MPC.updateStartPoint(res)
    evalPoints = RaceCourse.getSplinePositions(itpTrack, res, N)
    #println(evalPoints)
    tangentPoints = RaceCourse.computeGradientPoints_(itpOutBound, itpInBound, evalPoints, N)
    MPC.updateTangentPoints(tangentPoints)
    #println(res)
end

plot(prin_x, prin_y)
