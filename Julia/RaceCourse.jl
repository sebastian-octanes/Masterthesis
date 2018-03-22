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
     println("x", x)
     println("y", y)
     println("aOutBound", aOutBound)
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

function buildRaceTrack2(trackWidth, OriginX, OriginY)
    #do nothing
end

function addRightTurn(direction, radius, trackWidth, track, OutBound, InBound)
    radius_minus = radius - trackWidth/2.0
    radius_plus = radius + trackWidth/2.0
    last_pointX = track[end,1]
    last_pointY = track[end,2]
    new_points = Array{Float64}(4,2)
    new_points_out = Array{Float64}(4,2)
    new_points_in = Array{Float64}(4,2)
    if direction == 0
        for i in 1:4
            new_points[i, 1] = last_pointX + radius - radius*cos(i * pi/8)
            new_points[i, 2] = last_pointY + radius*sin(i * pi/8)
            new_points_out[i, 1] = last_pointX + radius - radius_plus * cos(i * pi/8)
            new_points_out[i, 2] = last_pointY + radius_plus*sin(i * pi/8)
            new_points_in[i, 1] = last_pointX + radius - radius_minus *cos(i * pi/8)
            new_points_in[i, 2] = last_pointY + radius_minus*sin(i * pi/8)
        end
    end
    if direction == 1
        for i in 1:4
            new_points[i, 1] = last_pointX + radius*sin(i * pi/8)
            new_points[i, 2] = last_pointY -radius + radius*cos(i * pi/8)
            new_points_out[i, 1] = last_pointX +  radius_plus * sin(i * pi/8)
            new_points_out[i, 2] = last_pointY -radius + radius_plus*cos(i * pi/8)
            new_points_in[i, 1] = last_pointX + radius_minus *sin(i * pi/8)
            new_points_in[i, 2] = last_pointY -radius + radius_minus*cos(i * pi/8)
        end
    end
    if direction == 2
        for i in 1:4
            new_points[i, 1] = last_pointX - radius + radius*cos(i * pi/8)
            new_points[i, 2] = last_pointY - radius*sin(i * pi/8)
            new_points_out[i, 1] = last_pointX - radius + radius_plus * cos(i * pi/8)
            new_points_out[i, 2] = last_pointY - radius_plus*sin(i * pi/8)
            new_points_in[i, 1] = last_pointX - radius + radius_minus *cos(i * pi/8)
            new_points_in[i, 2] = last_pointY - radius_minus*sin(i * pi/8)
        end
    end
    if direction == 3
        for i in 1:4
            new_points[i, 1] = last_pointX - radius - radius*sin(i * pi/8)
            new_points[i, 2] = last_pointY + radius - radius*cos(i * pi/8)
            new_points_out[i, 1] = last_pointX  - radius_plus * sin(i * pi/8)
            new_points_out[i, 2] = last_pointY + radius - radius_plus*cos(i * pi/8)
            new_points_in[i, 1] = last_pointX - radius_minus *sin(i * pi/8)
            new_points_in[i, 2] = last_pointY + radius - radius_minus*cos(i * pi/8)
        end
    end
    track = vcat(track, new_points)
    OutBound = vcat(OutBound, new_points_out)
    InBound = vcat(InBound, new_points_in)
    return track, OutBound, InBound
end

function addStraight(direction, leng, trackWidth, track, OutBound, InBound)
    #always add 4 points per straight
    #dont judge me on the "len" but apparently julia allows no length
    last_pointX = track[end,1]
    last_pointY = track[end,2]
    new_points = Array{Float64}(4,2)
    new_points_out = Array{Float64}(4,2)
    new_points_in = Array{Float64}(4,2)
    if direction == 0 #upwards
        for i in 1:4
            new_points[i,1] = last_pointX
            new_points[i,2] = last_pointY + leng * i / 4.0
            new_points_out[i,1] = last_pointX - trackWidth/ 2.0
            new_points_out[i,2] = last_pointY + i * leng / 4.0
            new_points_in[i,1] = last_pointX + trackWidth / 2.0
            new_points_in[i,2] = last_pointY + i * leng / 4.0
        end
    end
    if direction == 1  # right
        for i in 1:4
            new_points[i,1] = last_pointX + leng * i / 4.0
            new_points[i,2] = last_pointY
            new_points_out[i,1] = last_pointX + leng * i / 4.0
            new_points_out[i,2] = last_pointY + trackWidth/ 2.0
            new_points_in[i,1] = last_pointX + leng * i / 4.0
            new_points_in[i,2] = last_pointY - trackWidth/ 2.0
        end
    end
    if direction == 2  # downward
        for i in 1:4
            new_points[i,1] = last_pointX
            new_points[i,2] = last_pointY - leng * i / 4.0
            new_points_out[i,1] = last_pointX + trackWidth/ 2.0
            new_points_out[i,2] = last_pointY - i * leng / 4.0
            new_points_in[i,1] = last_pointX - trackWidth / 2.0
            new_points_in[i,2] = last_pointY - i * leng / 4.0
        end
    end
    if direction == 3  # left
        for i in 1:4
            new_points[i,1] = last_pointX - leng * i / 4.0
            new_points[i,2] = last_pointY
            new_points_out[i,1] = last_pointX - i *leng / 4.0
            new_points_out[i,2] = last_pointY - trackWidth / 2.0
            new_points_in[i,1] = last_pointX - i *leng / 4.0
            new_points_in[i,2] = last_pointY + trackWidth / 2.0
        end
    end

    track = vcat(track, new_points)
    OutBound = vcat(OutBound, new_points_out)
    InBound = vcat(InBound, new_points_in)
    return track, OutBound, InBound
end

function plotRaceTrack(itpTrack, itpOutBound, itpInBound)

    tfine = 0:.1:20

    xsT, ysT = [itpTrack[t,1] for t in tfine], [itpTrack[t,2] for t in tfine]
    xsO, ysO = [itpOutBound[t,1] for t in tfine], [itpOutBound[t,2] for t in tfine]
    xsI, ysI = [itpInBound[t,1] for t in tfine], [itpInBound[t,2] for t in tfine]
    println(ysT)
    plot(xsT, ysT, label="spline")
    plot(xsO, ysO, label="spline")
    plot(xsI, ysI, label="spline")
end
    export buildRaceTrack, computeGradientPoints, computeGradientAngle, getSplinePosition, addStraight, addRightTurn, plotRaceTrack
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







        #=
                if (direction == 1):  # right
                    last_point = self.track[-1]
                    for i in range(0, 4, 1):
                        new_point = [last_point[0] + length/4.0, last_point[1]]
                        self.track.append(new_point)
                        new_point_bnd_left = [last_point[0] + length/4.0, last_point[1] + self.track_width/2.0]
                        new_point_bnd_right= [last_point[0] + length/4.0, last_point[1] - self.track_width/2.0]
                        self.track_bounds.append([new_point_bnd_left[0], new_point_bnd_left[1], new_point_bnd_right[0], new_point_bnd_right[1]])
                        last_point = new_point
                if (direction == 2):  # downward
                    last_point = self.track[-1]
                    for i in range(0, 4, 1):
                        new_point = [last_point[0], last_point[1] - length/4.0]
                        self.track.append(new_point)
                        new_point_bnd_left = [last_point[0] + self.track_width/2.0, last_point[1] - length/4.0]
                        new_point_bnd_right= [last_point[0] - self.track_width/2.0, last_point[1] - length/4.0]
                        self.track_bounds.append([new_point_bnd_left[0], new_point_bnd_left[1], new_point_bnd_right[0], new_point_bnd_right[1]])
                        last_point = new_point
                if (direction == 3):  # left
                    last_point = self.track[-1]
                    for i in range(0, 4, 1):
                        new_point = [last_point[0] - length/4.0, last_point[1]]
                        self.track.append(new_point)
                        new_point_bnd_left = [last_point[0] - length/4.0, last_point[1] - self.track_width/2.0]
                        new_point_bnd_right= [last_point[0] - length/4.0, last_point[1] + self.track_width/2.0]
                        self.track_bounds.append([new_point_bnd_left[0], new_point_bnd_left[1], new_point_bnd_right[0], new_point_bnd_right[1]])
                        last_point = new_point
                        =#
