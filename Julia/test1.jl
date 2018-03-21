include("mpc.jl")
include("vehiclemodel.jl")
include("raceCourse.jl")
using VehicleModel
using MPC
using RaceCourse

N = 5
dt = 0.01
startPose = VehicleModel.CarPose(0,0,0.01,pi/2)
stateVector = []
itpTrack, itpOutBound, itpInBound = buildRaceTrack(15, 4, 15, 0)
start_=[startPose.x, startPose.y, startPose.v, startPose.yaw, 0, 0]
for i in 0:N
    stateVector = vcat(stateVector, start_) #add initial guess to vector
end

evalPoints = RaceCourse.getSplinePositions(itpTrack, stateVector, N)
tangentPoints = RaceCourse.computeGradientPoints_(itpOutBound, itpInBound, evalPoints, N)
m = MPC.initMPC(N, dt, startPose, tangentPoints)
#println(m)
for i in 1:10
    res = MPC.solveMPC()
    MPC.updateStartPoint(res)
    evalPoints = RaceCourse.getSplinePositions(itpTrack, res, N)
    #println(evalPoints)
    tangentPoints = RaceCourse.computeGradientPoints_(itpOutBound, itpInBound, evalPoints, N)
    println(tangentPoints)
    MPC.updateTangentPoints(tangentPoints)
    #println(MPC.m)
end
