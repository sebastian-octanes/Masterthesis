module RaceCourse

using Interpolations
using PyPlot


function computeGradientPoints(itpBound, evalPoint)
 x1, y1  = itpBound[evalPoint + 0.01,1], itpBound[evalPoint + 0.01,2]
 x2, y2  = itpBound[evalPoint - 0.01,1], itpBound[evalPoint - 0.01,2]
 return [x1, y1, x2, y2]
end
function computeGradientPoints_(itpOutBound, itpInBound ,evalPoint, N)
    tangentPoints = []
    for i in 1:N
        x1, y1  = itpOutBound[evalPoint[i] - 0.01,1], itpOutBound[evalPoint[i] - 0.01,2]
        x2, y2  = itpOutBound[evalPoint[i] + 0.01,1], itpOutBound[evalPoint[i] + 0.01,2]
        tangentPoints = vcat(tangentPoints, x1, y1, x2, y2)
        x1, y1  = itpInBound[evalPoint[i] - 0.01,1], itpInBound[evalPoint[i] - 0.01,2]
        x2, y2  = itpInBound[evalPoint[i] + 0.01,1], itpInBound[evalPoint[i] + 0.01,2]
        tangentPoints = vcat(tangentPoints, x1, y1, x2, y2)
    end
    return tangentPoints
end

function computeGradientAngle(itpBound, evalPoint)
 x1, y1  = itpBound[evalPoint - 0.01,1], itpBound[evalPoint - 0.01,2]
 x2, y2  = itpBound[evalPoint + 0.01,1], itpBound[evalPoint + 0.01,2]
 x3, y3  = x1 + 1, y1
 -1* (atan2(y3 - y1, x3 - x1)- atan2(y2 -y1, x2 -x1))
end

function getSplinePosition(itpBound, x, y)
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
 return indx
end

function getSplinePositions(itpBound, stateVector, N)
    ret=[]
    for j in 1:N
        x = stateVector[(j*6) + 1]
        y = stateVector[(j*6) + 2]
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

function buildRaceTrack(radius, trackWidth, OriginX, OriginY)

 t = 0:.1:1
 x = (radius + trackWidth/2) * sin.(2π*t) + OriginX
 y = (radius + trackWidth/2) * cos.(2π*t) + OriginY
 aOutBound = hcat(x,y)

 x = (radius - trackWidth/2) * sin.(2π*t) + OriginX
 y = (radius - trackWidth/2) * cos.(2π*t) + OriginY
 aInBound = hcat(x,y)

 x = radius * sin.(2π*t) + OriginX
 y = radius * cos.(2π*t) + OriginY
 aTrack = hcat(x,y)

 global itpTrack = scale(interpolate(aTrack, (BSpline(Cubic(Natural())), NoInterp()), OnGrid()), t, 1:2)
 global itpOutBound = scale(interpolate(aOutBound, (BSpline(Cubic(Natural())), NoInterp()), OnGrid()), t, 1:2)
 global itpInBound = scale(interpolate(aInBound, (BSpline(Cubic(Natural())), NoInterp()), OnGrid()), t, 1:2)
 return itpTrack, itpOutBound, itpInBound
end

function plotRaceTrack(itpTrack, itpOutBound, itpInBound)

 tfine = 0:.01:1

 xsT, ysT = [itpTrack[t,1] for t in tfine], [itpTrack[t,2] for t in tfine]
 xsO, ysO = [itpOutBound[t,1] for t in tfine], [itpOutBound[t,2] for t in tfine]
 xsI, ysI = [itpInBound[t,1] for t in tfine], [itpInBound[t,2] for t in tfine]

 plot(xsT, ysT, label="spline")
 plot(xsO, ysO, label="spline")
 plot(xsI, ysI, label="spline")
end

export plotRaceTrack, buildRaceTrack, computeGradientPoints, computeGradientAngle, getSplinePosition

end

#=
using RaceCourse
using PyPlot

itpTrack, itpOutBound, itpInBound = buildRaceTrack(100, 50)
plotRaceTrack(itpTrack, itpOutBound, itpInBound)
x = computeGradientPoints(itpTrack, 0.1)
alpha = computeGradientAngle(itpTrack, 0.1)
println(alpha)
pos = getSplinePosition(itpTrack, 100, 10)
println(pos)
plot(x[1:2:4], x[2:2:4])
=#
