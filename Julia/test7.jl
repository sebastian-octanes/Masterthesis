include("RaceCourse.jl")
using Interpolations

trackWidth = 2
radius = 10.0
leng = 10.0
direction = 0

track = zeros(Float64,1,2)
OutBound = zeros(Float64,1,2)
InBound = zeros(Float64,1,2)
OutBound[1,1] = - trackWidth/2
InBound[1,1] = + trackWidth/2


track, OutBound, InBound = RaceCourse.addStraight(0, leng, trackWidth, track, OutBound, InBound)
#track, OutBound, InBound = RaceCourse.addRightTurn(0, radius, trackWidth, track, OutBound, InBound)
#track, OutBound, InBound = RaceCourse.addRightTurn(1, radius, trackWidth, track, OutBound, InBound)
#track, OutBound, InBound = RaceCourse.addRightTurn(2, radius, trackWidth, track, OutBound, InBound)
#track, OutBound, InBound = RaceCourse.addRightTurn(3, radius, trackWidth, track, OutBound, InBound)


#track, OutBound, InBound = RaceCourse.addStraight(1, leng, trackWidth, track, OutBound, InBound)
t = 1:1:length(track)/2

itpTrack = scale(interpolate(track, (BSpline(Cubic(Natural())), NoInterp()), OnGrid()), t, 1:2)
itpOutBound = scale(interpolate(OutBound, (BSpline(Cubic(Natural())), NoInterp()), OnGrid()), t, 1:2)
itpInBound = scale(interpolate(InBound, (BSpline(Cubic(Natural())), NoInterp()), OnGrid()), t, 1:2)


RaceCourse.plotRaceTrack(itpTrack, itpOutBound, itpInBound)

println("track", track)
#println("OutBound", OutBound)
#println("InBound", InBound)
