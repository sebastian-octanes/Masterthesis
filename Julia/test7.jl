include("RaceCourse.jl")
using Interpolations
using PyPlot

trackWidth = 2
radius = 4.0
leng = 10.0
direction = 0


track = zeros(Float64,1,2)
leftBound = zeros(Float64,1,2)
rightBound = zeros(Float64,1,2)
leftBound[1,1] = - trackWidth/2
rightBound[1,1] = + trackWidth/2


track, leftBound, rightBound = RaceCourse.addStraight(0, 10, trackWidth, track, leftBound, rightBound)
track, leftBound, rightBound = RaceCourse.addRightTurn(0, 5, trackWidth, track, leftBound, rightBound)
track, leftBound, rightBound = RaceCourse.addRightTurn(1, 4, trackWidth, track, leftBound, rightBound)
track, leftBound, rightBound = RaceCourse.addLeftTurn(2, 5, trackWidth, track, leftBound, rightBound)
track, leftBound, rightBound = RaceCourse.addStraight(1, 4, trackWidth, track, leftBound, rightBound)
track, leftBound, rightBound = RaceCourse.addLeftTurn(3, 5, trackWidth, track, leftBound, rightBound)
track, leftBound, rightBound = RaceCourse.addRightTurn(0, 7, trackWidth, track, leftBound, rightBound)
track, leftBound, rightBound = RaceCourse.addRightTurn(1, 7, trackWidth, track, leftBound, rightBound)
track, leftBound, rightBound = RaceCourse.addRightTurn(2, 4, trackWidth, track, leftBound, rightBound)
track, leftBound, rightBound = RaceCourse.addLeftTurn(1, 4, trackWidth, track, leftBound, rightBound)
track, leftBound, rightBound = RaceCourse.addStraight(2, 7, trackWidth, track, leftBound, rightBound)
track, leftBound, rightBound = RaceCourse.addRightTurn(2, 2, trackWidth, track, leftBound, rightBound)
track, leftBound, rightBound = RaceCourse.addRightTurn(3, 2, trackWidth, track, leftBound, rightBound)
track, leftBound, rightBound = RaceCourse.addLeftTurn(0, 3, trackWidth, track, leftBound, rightBound)
track, leftBound, rightBound = RaceCourse.addStraight(3, 8, trackWidth, track, leftBound, rightBound)
track, leftBound, rightBound = RaceCourse.addLeftTurn(1, 5, trackWidth, track, leftBound, rightBound)
track, leftBound, rightBound = RaceCourse.addRightTurn(2, 5, trackWidth, track, leftBound, rightBound)
track, leftBound, rightBound = RaceCourse.addRightTurn(3, 4, trackWidth, track, leftBound, rightBound)
track, leftBound, rightBound = RaceCourse.addStraight(0, 5.5, trackWidth, track, leftBound, rightBound)

#track, leftBound, rightBound = RaceCourse.addRightTurn(2, radius, trackWidth, track, leftBound, rightBound)
#track, leftBound, rightBound = RaceCourse.addRightTurn(3, radius, trackWidth, track, leftBound, rightBound)
#track, leftBound, rightBound = RaceCourse.addRightTurn(2, radius, trackWidth, track, leftBound, rightBound)
#track, leftBound, rightBound = RaceCourse.addRightTurn(3, radius, trackWidth, track, leftBound, rightBound)

println("track", leftBound)

x = length(track)/2
step = 1/x
t = 0:step:(1- step)

itpTrack = scale(interpolate(track, (BSpline(Cubic(Natural())), NoInterp()), OnGrid()), t, 1:2)
itpLeftBound = scale(interpolate(leftBound, (BSpline(Cubic(Natural())), NoInterp()), OnGrid()), t, 1:2)
itpRightBound = scale(interpolate(rightBound, (BSpline(Cubic(Natural())), NoInterp()), OnGrid()), t, 1:2)


RaceCourse.plotRaceTrack(itpTrack, itpLeftBound, itpRightBound)
