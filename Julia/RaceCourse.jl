module RaceCourse

using Interpolations
using PyPlot


function computeGradientPoints(itpBound, evalPoint)
    x1, y1  = itpBound[evalPoint + 0.01,1], itpBound[evalPoint + 0.01,2]
    x2, y2  = itpBound[evalPoint - 0.01,1], itpBound[evalPoint - 0.01,2]
    return [x1, y1, x2, y2]
end
function computeGradientPoints_(itpLeftBound, itpRightBound ,evalPoint, N)
    tangentPoints = []
    for i in 1:N
        x1, y1  = itpLeftBound[evalPoint[i] - 0.01,1], itpLeftBound[evalPoint[i] - 0.01,2]
        x2, y2  = itpLeftBound[evalPoint[i] + 0.01,1], itpLeftBound[evalPoint[i] + 0.01,2]
        tangentPoints = vcat(tangentPoints, x1, y1, x2, y2)
        x1, y1  = itpRightBound[evalPoint[i] - 0.01,1], itpRightBound[evalPoint[i] - 0.01,2]
        x2, y2  = itpRightBound[evalPoint[i] + 0.01,1], itpRightBound[evalPoint[i] + 0.01,2]
        tangentPoints = vcat(tangentPoints, x1, y1, x2, y2)
    end
    return tangentPoints
end

function getTrackPoints(itpTrack, itpLeftBound, itpRightBound, evalPoints, N)
    trackPoints = []
    for i in 1:N
        x0, y0 = itpTrack[evalPoints[i],1], itpTrack[evalPoints[i],2]
        x1, y1 = itpLeftBound[evalPoints[i],1], itpLeftBound[evalPoints[i],2]
        x2, y2 = itpRightBound[evalPoints[i],1], itpRightBound[evalPoints[i],2]
        trackPoints = vcat(trackPoints, x0, y0, x1, y1, x2, y2)
    end
    return trackPoints
end

function getForwardTrackPoint(itpTrack, evalPoints, N)
    x1, y1 = itpTrack[evalPoints[N] + 0.02, 1], itpTrack[evalPoints[N] + 0.02, 2]
    return [x1, y1]
end

function getMidTrackPoints(itpTrack, evalPoints, N)
    midTrackPoints = []
    for i in 1:N
        x1, y1 = itpTrack[evalPoints[i],1], itpTrack[evalPoints[i],2]
        midTrackPoints = vcat(midTrackPoints, x1, y1)
    end
    return midTrackPoints
end
function computeGradientAngle(itpBound, evalPoint)
     x1, y1  = itpBound[evalPoint - 0.01,1], itpBound[evalPoint - 0.01,2]
     x2, y2  = itpBound[evalPoint + 0.01,1], itpBound[evalPoint + 0.01,2]
     x3, y3  = x1 + 1, y1
    -1* (atan2(y3 - y1, x3 - x1)- atan2(y2 -y1, x2 -x1))
end

function getSplinePosition(itpBound, x, y)
     #t = 0:0.0005:1
     smallest = 1000
     indx = 0
         for i = 0:0.0005:1
             x1, y1  = itpBound[i,1], itpBound[i,2]
             dist = sqrt((x1-x)^2 + (y - y1)^2)
             if dist < smallest
                 smallest = dist
                 indx = i
             end
         end
     return indx
end

function getSplinePositions(itpBound, stateVector, N)
    ret=[]
    for j in 1:N
        x = stateVector[(j*8) + 1]
        y = stateVector[(j*8) + 2]
        t = 0:0.001:1
        smallest = 1000
        indx = 0
        for i = 0:0.001:1
            x1, y1  = itpBound[i,1], itpBound[i,2]
            dist = sqrt((x1-x)^2 + (y - y1)^2)
            if dist < smallest
                smallest = dist
                indx = i
            end
        end
        ret = vcat(ret, indx)
    end
    return ret
end

function computeDistToTrackBoarder(itpTrack, itpLeftBound, carPose)
    x, y = carPose.x, carPose.y
    indx = getSplinePosition(itpTrack, carPose.x, carPose.y)
    x0, y0 = itpTrack[indx, 1], itpTrack[indx, 2]
    x1, y1 = itpLeftBound[indx, 1], itpLeftBound[indx, 2]
    ax = x - x0
    ay = y - y0
    ab1 =  ax*(x1 - x0)+ ay*(y1 - y0)
    B1 =   sqrt((x1 - x0)^2 + (y1 - y0)^2)
    dist = ab1/B1
    return dist
end

function computeDistToTrackStartX(itpTrack, itpLeftBound, carPose)
    x, y = carPose.x, carPose.y

    x0, y0 = 0, 0
    x1, y1 = 2, 0
    ax = x - x0
    ay = y - y0
    ab1 =  ax*(x1 - x0)+ ay*(y1 - y0)
    B1 =   sqrt((x1 - x0)^2 + (y1 - y0)^2)
    dist = abs(ab1/B1)
    return dist
end

function computeDistToTrackStartY(carPose)
    y  =  carPose.y
    y0 = 0
    dist = abs(y - y0)
    return dist
end

function buildRaceTrack(radius, trackWidth, OriginX, OriginY)

     t = 0:.1:1
     x = (radius + trackWidth/2) * sin.(2π*t) + OriginX
     y = (radius + trackWidth/2) * cos.(2π*t) + OriginY
     aOutBound = hcat(x,y)
     println("x", x)
     println("y", y)
     println("aOutBound", aOutBound)
     x = (radius - trackWidth/2) * sin.(2π*t) + OriginX
     y = (radius - trackWidth/2) * cos.(2π*t) + OriginY
     aInBound = hcat(x,y)

     x = radius * sin.(2π*t) + OriginX
     y = radius * cos.(2π*t) + OriginY
     aTrack = hcat(x,y)

     itpTrack = scale(interpolate(aTrack, (BSpline(Cubic(Natural())), NoInterp()), OnGrid()), t, 1:2)
     itpOutBound = scale(interpolate(aOutBound, (BSpline(Cubic(Natural())), NoInterp()), OnGrid()), t, 1:2)
     itpInBound = scale(interpolate(aInBound, (BSpline(Cubic(Natural())), NoInterp()), OnGrid()), t, 1:2)
     return itpTrack, itpOutBound, itpInBound
end

function buildRaceTrack2(trackWidth)

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
    track, leftBound, rightBound = RaceCourse.addRightTurn(2, 3, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addRightTurn(3, 3, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addLeftTurn(0, 3, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addStraight(3, 6, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addLeftTurn(1, 5, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addRightTurn(2, 5, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addRightTurn(3, 4, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addStraight(0, 6, trackWidth, track, leftBound, rightBound)

    x = length(track)/2
    step = 1/x
    t = 0:step:(1- step)

    itpTrack = scale(interpolate(track, (BSpline(Cubic(Natural())), NoInterp()), OnGrid()), t, 1:2)
    itpLeftBound = scale(interpolate(leftBound, (BSpline(Cubic(Natural())), NoInterp()), OnGrid()), t, 1:2)
    itpRightBound = scale(interpolate(rightBound, (BSpline(Cubic(Natural())), NoInterp()), OnGrid()), t, 1:2)
    return itpTrack, itpLeftBound, itpRightBound
end


function buildRaceTrack3(trackWidth)

    track = zeros(Float64,1,2)
    leftBound = zeros(Float64,1,2)
    rightBound = zeros(Float64,1,2)
    leftBound[1,1] = - trackWidth/2
    rightBound[1,1] = + trackWidth/2

    track, leftBound, rightBound = RaceCourse.addStraight(0, 15, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addRightTurn(0, 7, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addRightTurn(1, 7, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addLeftTurn(2, 7, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addStraight(1, 20, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addRightTurn(1, 7, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addRightTurn(2, 4, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addLeftTurn(1, 4, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addStraight(2, 7, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addRightTurn(2, 3, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addStraight(3, 20, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addRightTurn(3, 5, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addLeftTurn(0, 5, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addStraight(3, 2.7, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addRightTurn(3, 4, trackWidth, track, leftBound, rightBound)

    x = length(track)/2
    step = 1/x
    t = 0:step:(1- step)

    itpTrack = scale(interpolate(track, (BSpline(Cubic(Natural())), NoInterp()), OnGrid()), t, 1:2)
    itpLeftBound = scale(interpolate(leftBound, (BSpline(Cubic(Natural())), NoInterp()), OnGrid()), t, 1:2)
    itpRightBound = scale(interpolate(rightBound, (BSpline(Cubic(Natural())), NoInterp()), OnGrid()), t, 1:2)
    return itpTrack, itpLeftBound, itpRightBound
end


function buildRaceTrack4(trackWidth)

    track = zeros(Float64,1,2)
    leftBound = zeros(Float64,1,2)
    rightBound = zeros(Float64,1,2)
    leftBound[1,1] = - trackWidth/2
    rightBound[1,1] = + trackWidth/2

    track, leftBound, rightBound = RaceCourse.addStraight(0, 15, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addRightTurn(0, 7, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addRightTurn(1, 7, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addLeftTurn(2, 7, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addStraight(1, 20, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addRightTurn(1, 7, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addRightTurn(2, 6, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addLeftTurn(1, 6, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addStraight(2, 7, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addRightTurn(2, 5, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addStraight(3, 16, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addRightTurn(3, 5, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addLeftTurn(0, 5, trackWidth, track, leftBound, rightBound)
    #track, leftBound, rightBound = RaceCourse.addStraight(3, 2.7, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addRightTurn(3, 5, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addStraight(0, 4, trackWidth, track, leftBound, rightBound)

    x = length(track)/2
    step = 1/x
    t = 0:step:(1- step)

    itpTrack = scale(interpolate(track, (BSpline(Cubic(Natural())), NoInterp()), OnGrid()), t, 1:2)
    itpLeftBound = scale(interpolate(leftBound, (BSpline(Cubic(Natural())), NoInterp()), OnGrid()), t, 1:2)
    itpRightBound = scale(interpolate(rightBound, (BSpline(Cubic(Natural())), NoInterp()), OnGrid()), t, 1:2)
    return itpTrack, itpLeftBound, itpRightBound
end
function buildRaceTrackCircle(trackWidth)

    track = zeros(Float64,1,2)
    leftBound = zeros(Float64,1,2)
    rightBound = zeros(Float64,1,2)
    leftBound[1,1] = - trackWidth/2
    rightBound[1,1] = + trackWidth/2


    track, leftBound, rightBound = RaceCourse.addRightTurn(0, 7, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addRightTurn(1, 7, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addRightTurn(2, 7, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addRightTurn(3, 7, trackWidth, track, leftBound, rightBound)

    x = length(track)/2
    step = 1/x
    t = 0:step:(1- step)

    itpTrack = scale(interpolate(track, (BSpline(Cubic(Natural())), NoInterp()), OnGrid()), t, 1:2)
    itpLeftBound = scale(interpolate(leftBound, (BSpline(Cubic(Natural())), NoInterp()), OnGrid()), t, 1:2)
    itpRightBound = scale(interpolate(rightBound, (BSpline(Cubic(Natural())), NoInterp()), OnGrid()), t, 1:2)
    return itpTrack, itpLeftBound, itpRightBound
end

function buildRaceTrackStraight(trackWidth)

    track = zeros(Float64,1,2)
    leftBound = zeros(Float64,1,2)
    rightBound = zeros(Float64,1,2)
    leftBound[1,1] = - trackWidth/2
    rightBound[1,1] = + trackWidth/2

    track, leftBound, rightBound = RaceCourse.addRightTurn(0, 7, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addRightTurn(1, 7, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addStraight(2, 15, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addRightTurn(2, 7, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addRightTurn(3, 7, trackWidth, track, leftBound, rightBound)
    track, leftBound, rightBound = RaceCourse.addStraight(0, 10, trackWidth, track, leftBound, rightBound)
    x = length(track)/2
    step = 1/x
    t = 0:step:(1- step)

    itpTrack = scale(interpolate(track, (BSpline(Cubic(Natural())), NoInterp()), OnGrid()), t, 1:2)
    itpLeftBound = scale(interpolate(leftBound, (BSpline(Cubic(Natural())), NoInterp()), OnGrid()), t, 1:2)
    itpRightBound = scale(interpolate(rightBound, (BSpline(Cubic(Natural())), NoInterp()), OnGrid()), t, 1:2)

    return itpTrack, itpLeftBound, itpRightBound
end

function addRightTurn(direction, radius, trackWidth, track, leftBound, rightBound)
    radius_minus = radius - trackWidth/2.0
    radius_plus = radius + trackWidth/2.0
    last_pointX = track[end,1]
    last_pointY = track[end,2]
    newPoints = Array{Float64}(4,2)
    newPointsLeft = Array{Float64}(4,2)
    newPointsRight = Array{Float64}(4,2)
    if direction == 0
        for i in 1:4
            newPoints[i, 1] = last_pointX + radius - radius*cos(i * pi/8)
            newPoints[i, 2] = last_pointY + radius*sin(i * pi/8)
            newPointsLeft[i, 1] = last_pointX + radius - radius_plus * cos(i * pi/8)
            newPointsLeft[i, 2] = last_pointY + radius_plus*sin(i * pi/8)
            newPointsRight[i, 1] = last_pointX + radius - radius_minus *cos(i * pi/8)
            newPointsRight[i, 2] = last_pointY + radius_minus*sin(i * pi/8)
        end
    end
    if direction == 1
        for i in 1:4
            newPoints[i, 1] = last_pointX + radius*sin(i * pi/8)
            newPoints[i, 2] = last_pointY -radius + radius*cos(i * pi/8)
            newPointsLeft[i, 1] = last_pointX +  radius_plus * sin(i * pi/8)
            newPointsLeft[i, 2] = last_pointY -radius + radius_plus*cos(i * pi/8)
            newPointsRight[i, 1] = last_pointX + radius_minus *sin(i * pi/8)
            newPointsRight[i, 2] = last_pointY -radius + radius_minus*cos(i * pi/8)
        end
    end
    if direction == 2
        for i in 1:4
            newPoints[i, 1] = last_pointX - radius + radius*cos(i * pi/8)
            newPoints[i, 2] = last_pointY - radius*sin(i * pi/8)
            newPointsLeft[i, 1] = last_pointX - radius + radius_plus * cos(i * pi/8)
            newPointsLeft[i, 2] = last_pointY - radius_plus*sin(i * pi/8)
            newPointsRight[i, 1] = last_pointX - radius + radius_minus *cos(i * pi/8)
            newPointsRight[i, 2] = last_pointY - radius_minus*sin(i * pi/8)
        end
    end
    if direction == 3
        for i in 1:4
            newPoints[i, 1] = last_pointX - radius*sin(i * pi/8)
            newPoints[i, 2] = last_pointY + radius - radius*cos(i * pi/8)
            newPointsLeft[i, 1] = last_pointX  - radius_plus * sin(i * pi/8)
            newPointsLeft[i, 2] = last_pointY + radius - radius_plus*cos(i * pi/8)
            newPointsRight[i, 1] = last_pointX - radius_minus *sin(i * pi/8)
            newPointsRight[i, 2] = last_pointY + radius - radius_minus*cos(i * pi/8)
        end
    end
    track = vcat(track, newPoints)
    leftBound = vcat(leftBound, newPointsLeft)
    rightBound = vcat(rightBound, newPointsRight)
    return track, leftBound, rightBound
end

function addLeftTurn(direction, radius, trackWidth, track, leftBound, rightBound)
    radius_minus = radius - trackWidth/2.0
    radius_plus = radius + trackWidth/2.0
    last_pointX = track[end,1]
    last_pointY = track[end,2]
    newPoints = Array{Float64}(4,2)
    newPointsLeft = Array{Float64}(4,2)
    newPointsRight = Array{Float64}(4,2)
    if direction == 0
        for i in 1:4
            newPoints[i, 1] = last_pointX - radius + radius*cos(i * pi/8)
            newPoints[i, 2] = last_pointY + radius*sin(i * pi/8)
            newPointsRight[i, 1] = last_pointX - radius + radius_plus * cos(i * pi/8)
            newPointsRight[i, 2] = last_pointY + radius_plus*sin(i * pi/8)
            newPointsLeft[i, 1] = last_pointX - radius + radius_minus *cos(i * pi/8)
            newPointsLeft[i, 2] = last_pointY + radius_minus*sin(i * pi/8)
        end
    end
    if direction == 1
        for i in 1:4
            newPoints[i, 1] = last_pointX - radius*sin(i * pi/8)
            newPoints[i, 2] = last_pointY -radius + radius*cos(i * pi/8)
            newPointsRight[i, 1] = last_pointX -  radius_plus * sin(i * pi/8)
            newPointsRight[i, 2] = last_pointY -radius + radius_plus*cos(i * pi/8)
            newPointsLeft[i, 1] = last_pointX - radius_minus *sin(i * pi/8)
            newPointsLeft[i, 2] = last_pointY -radius + radius_minus*cos(i * pi/8)
        end
    end
    if direction == 2
        for i in 1:4
            newPoints[i, 1] = last_pointX + radius - radius*cos(i * pi/8)
            newPoints[i, 2] = last_pointY - radius*sin(i * pi/8)
            newPointsRight[i, 1] = last_pointX + radius - radius_plus * cos(i * pi/8)
            newPointsRight[i, 2] = last_pointY - radius_plus*sin(i * pi/8)
            newPointsLeft[i, 1] = last_pointX + radius - radius_minus *cos(i * pi/8)
            newPointsLeft[i, 2] = last_pointY - radius_minus*sin(i * pi/8)
        end
    end
    if direction == 3
        for i in 1:4
            newPoints[i, 1] = last_pointX + radius*sin(i * pi/8)
            newPoints[i, 2] = last_pointY + radius - radius*cos(i * pi/8)
            newPointsRight[i, 1] = last_pointX  + radius_plus * sin(i * pi/8)
            newPointsRight[i, 2] = last_pointY + radius - radius_plus*cos(i * pi/8)
            newPointsLeft[i, 1] = last_pointX + radius_minus *sin(i * pi/8)
            newPointsLeft[i, 2] = last_pointY + radius - radius_minus*cos(i * pi/8)
        end
    end
    track = vcat(track, newPoints)
    leftBound = vcat(leftBound, newPointsLeft)
    rightBound = vcat(rightBound, newPointsRight)
    return track, leftBound, rightBound
end


function addStraight(direction, leng, trackWidth, track, leftBound, rightBound)
    #always add 4 points per straight
    #dont judge me on the "len" but apparently julia allows no length
    last_pointX = track[end,1]
    last_pointY = track[end,2]
    newPoints = Array{Float64}(4,2)
    newPointsLeft = Array{Float64}(4,2)
    newPointsRight = Array{Float64}(4,2)
    if direction == 0 #upwards
        for i in 1:4
            newPoints[i,1] = last_pointX
            newPoints[i,2] = last_pointY + leng * i / 4.0
            newPointsLeft[i,1] = last_pointX - trackWidth/ 2.0
            newPointsLeft[i,2] = last_pointY + i * leng / 4.0
            newPointsRight[i,1] = last_pointX + trackWidth / 2.0
            newPointsRight[i,2] = last_pointY + i * leng / 4.0
        end
    end
    if direction == 1  # right
        for i in 1:4
            newPoints[i,1] = last_pointX + leng * i / 4.0
            newPoints[i,2] = last_pointY
            newPointsLeft[i,1] = last_pointX + leng * i / 4.0
            newPointsLeft[i,2] = last_pointY + trackWidth/ 2.0
            newPointsRight[i,1] = last_pointX + leng * i / 4.0
            newPointsRight[i,2] = last_pointY - trackWidth/ 2.0
        end
    end
    if direction == 2  # downward
        for i in 1:4
            newPoints[i,1] = last_pointX
            newPoints[i,2] = last_pointY - leng * i / 4.0
            newPointsLeft[i,1] = last_pointX + trackWidth/ 2.0
            newPointsLeft[i,2] = last_pointY - i * leng / 4.0
            newPointsRight[i,1] = last_pointX - trackWidth / 2.0
            newPointsRight[i,2] = last_pointY - i * leng / 4.0
        end
    end
    if direction == 3  # left
        for i in 1:4
            newPoints[i,1] = last_pointX - leng * i / 4.0
            newPoints[i,2] = last_pointY
            newPointsLeft[i,1] = last_pointX - i *leng / 4.0
            newPointsLeft[i,2] = last_pointY - trackWidth / 2.0
            newPointsRight[i,1] = last_pointX - i *leng / 4.0
            newPointsRight[i,2] = last_pointY + trackWidth / 2.0
        end
    end

    track = vcat(track, newPoints)
    leftBound = vcat(leftBound, newPointsLeft)
    rightBound = vcat(rightBound, newPointsRight)
    return track, leftBound, rightBound
end

function plotRaceTrack(itpTrack, leftBound, rightBound)

    tfine = 0:.001:1

    xsT, ysT = [itpTrack[t,1] for t in tfine], [itpTrack[t,2] for t in tfine]
    xsO, ysO = [leftBound[t,1] for t in tfine], [leftBound[t,2] for t in tfine]
    xsI, ysI = [rightBound[t,1] for t in tfine], [rightBound[t,2] for t in tfine]
    plot(xsT, ysT, label="spline")
    plot(xsO, ysO, label="spline")
    plot(xsI, ysI, label="spline")
end
    export buildRaceTrack, computeGradientPoints, getTrackPoints, computeGradientAngle, getSplinePosition, addStraight, addRightTurn, plotRaceTrack, buildRaceTrack2
end
