using SFML
using PyPlot
include("RaceCourse.jl")
include("VehicleModel.jl")
include("MPC.jl")
include("circular_buffer.jl")
include("smallMPC.jl")
#using CircBuffer
#using VehicleModel.CarPose, VehicleModel.CarState, VehicleModel
#using RaceCourse

mutable struct KeyControls
    up::Int
    down::Int
    left::Int
    right::Int
    reset::Int
end

function createcarsprite(carScaleX, carScaleY)
    texture = Texture("race_car.jpg")
    set_smooth(texture, true)
    carSizeX = 2 #meter
    carSizeY = 1.5 #meter

    rect = RectangleShape()
    set_texture(rect, texture)
    set_position(rect, Vector2f(500,250))
    set_size(rect, Vector2f(carScaleX * carSizeX,carScaleY * carSizeY))
    set_origin(rect, Vector2f(carScaleX * carSizeX/2, carScaleY*carSizeY/2))
    #rotate(rect, -90)
    return rect
end

function createRaceCourse(scaleX, scaleY, radius, offsetX, offsetY)
    circle = CircleShape()
    set_radius(circle, radius*scaleX)
    set_fillcolor(circle, SFML.white)
    set_position(circle, Vector2f(offsetX*scaleX + 15*scaleX, offsetY*scaleY))
    set_origin(circle, Vector2f(radius*scaleX, radius*scaleX))
    set_outline_thickness(circle, 2)
    set_outlinecolor(circle, SFML.red)
    return circle
end

function createRaceCourse2(scaleX, scaleY, offsetX, offsetY, itpTrack, itpLeftBound, itpRightBound, window)
    shape = ConvexShape()
    shapeLeft = ConvexShape()
    shapeRight = ConvexShape()

    N = 400
    #set_pointcount(shape, N)
    #set_outline_thickness(shape, 2)
    #set_outlinecolor(shape, SFML.red)
    set_pointcount(shapeLeft, N)
    set_outline_thickness(shapeLeft, 1)
    set_outlinecolor(shapeLeft, SFML.black)
    set_pointcount(shapeRight, N)
    set_outline_thickness(shapeRight, 1)
    set_outlinecolor(shapeRight, SFML.black)

    for i in 0:(N-1)
        #x = itpTrack[1/N * i, 1]
        #y = itpTrack[1/N * i, 2]
        #set_point(shape, i, Vector2f(offsetX*scaleX + x*scaleX , offsetY*scaleY - scaleY *y))
        x = itpLeftBound[1/N * i, 1]
        y = itpLeftBound[1/N * i, 2]
        set_point(shapeLeft, i, Vector2f(offsetX*scaleX + x*scaleX , offsetY*scaleY - scaleY *y))
        x = itpRightBound[1/N * i, 1]
        y = itpRightBound[1/N * i, 2]
        set_point(shapeRight, i, Vector2f(offsetX*scaleX + x*scaleX , offsetY*scaleY - scaleY *y))
        #set_points
    end
    return shapeLeft, shapeRight

end
function drawRaceCourse2(window, shapeLeft, shapeRight)
    draw(window, shapeLeft)
    #draw(window, shape)
    draw(window, shapeRight)
end
function createCarPathPoint(carPathBuffer, scaleX, scaleY, offsetX, offsetY, window, color)
    circle = CircleShape()
    set_radius(circle, 1)
    set_origin(circle, Vector2f(1, 1))
    for i in 1:length(carPathBuffer)
        carState = carPathBuffer[i]
        set_fillcolor(circle, color)
        set_position(circle, Vector2f(offsetX*scaleX + carState.x*scaleX, offsetY*scaleY - carState.y*scaleY))
        draw(window, circle)
    end
end
function createPredictionPoints(res, scaleX, scaleY, offsetX, offsetY, window, N)
    circle = CircleShape()
    set_radius(circle, 2)
    set_fillcolor(circle, SFML.blue)
    set_origin(circle, Vector2f(1, 1))
    #set_outlinecolor(circle, SFML.red)
    #set_outline_thickness(circle, 2)
    for i in 1:N-1
        x = res[i*8 + 1]
        y = - res[i*8 + 2]
        set_position(circle, Vector2f(offsetX*scaleX + x*scaleX, offsetY*scaleY + y*scaleY))
        draw(window, circle)
    end
    x = res[(N-1)*8 + 1]
    y = - res[(N-1)*8 + 2]
    set_position(circle, Vector2f(offsetX*scaleX + x*scaleX, offsetY*scaleY + y*scaleY))
    set_fillcolor(circle, SFML.red)
    draw(window, circle)
end

function createTangent(stateVector, itpTrack, itpLeftBound, itpRightBound, scaleX, scaleY, offsetX, offsetY, window)
    for j in 1:6:N
        carPose = VehicleModel.CarPose(stateVector[8*j + 1], stateVector[8*j + 2], stateVector[8*j + 3], stateVector[8*j + 4],stateVector[8*j + 5],stateVector[8*j + 6])

        i = RaceCourse.getSplinePosition(itpTrack, carPose.x, carPose.y)
        x =  RaceCourse.computeGradientPoints(itpLeftBound, i)
        alpha = RaceCourse.computeGradientAngle(itpLeftBound, i)
        line = RectangleShape()
        set_size(line, Vector2f(100, 1))
        set_outline_thickness(line, 1)
        rotate(line, - alpha *180/pi)
        set_fillcolor(line, SFML.green)
        set_origin(line, Vector2f(50, 1.5))
        off = Vector2f(offsetX * scaleX + x[1] * scaleX , offsetY * scaleY - x[2] * scaleY )
        set_position(line, off)
        draw(window, line)

        x =  RaceCourse.computeGradientPoints(itpRightBound, i)
        alpha = RaceCourse.computeGradientAngle(itpRightBound, i)
        line = RectangleShape()
        set_size(line, Vector2f(100, 1))
        set_outline_thickness(line, 1)
        rotate(line, - alpha *180/pi)
        set_fillcolor(line, SFML.green)
        set_origin(line, Vector2f(50, 1.5))
        off = Vector2f(offsetX * scaleX + x[1] * scaleX , offsetY * scaleY - x[2] * scaleY )
        set_position(line, off)
        draw(window, line)

    end
    carPose = VehicleModel.CarPose(stateVector[8*(N-1) + 1], stateVector[8*(N-1) + 2], stateVector[8*(N-1) + 3], stateVector[8*(N-1) + 4], 0, 0)

    i = RaceCourse.getSplinePosition(itpTrack, carPose.x, carPose.y)
    x =  RaceCourse.computeGradientPoints(itpLeftBound, i)
    alpha = RaceCourse.computeGradientAngle(itpLeftBound, i)
    line = RectangleShape()
    set_size(line, Vector2f(100, 1))
    set_outline_thickness(line, 1)
    rotate(line, - alpha *180/pi)
    set_fillcolor(line, SFML.cyan)
    set_origin(line, Vector2f(50, 1.5))
    off = Vector2f(offsetX * scaleX + x[1] * scaleX , offsetY * scaleY - x[2] * scaleY )
    set_position(line, off)
    draw(window, line)

    x =  RaceCourse.computeGradientPoints(itpRightBound, i)
    alpha = RaceCourse.computeGradientAngle(itpRightBound, i)
    line = RectangleShape()
    set_size(line, Vector2f(100, 1))
    set_outline_thickness(line, 1)
    rotate(line, - alpha *180/pi)
    set_fillcolor(line, SFML.cyan)
    set_origin(line, Vector2f(50, 1.5))
    off = Vector2f(offsetX * scaleX + x[1] * scaleX , offsetY * scaleY - x[2] * scaleY )
    set_position(line, off)
    draw(window, line)

end

function displayCarData(res, window)
    #text = TextRegular()
    text = RenderText()
    speed = res[3] * 3.61
    set_string(text, "x_d $(speed) km/h")
    set_color(text, SFML.red)
    set_charactersize(text, 25)
    set_position(text, Vector2f(50,450))
    draw(window, text)
end

function setpositioncar(carsprite, carPose, scaleX, scaleY, offsetX, offsetY)
    set_position(carsprite, Vector2f(scaleX * carPose.x + offsetX*scaleX, (- scaleY *carPose.y + offsetY*scaleY)))
    rotate(carsprite, -(carPose.psi )*180/pi)
end

function checkkeys()
    keyControls = KeyControls(0,0,0,0,0)
    if is_key_pressed(KeyCode.UP)
        keyControls.up = 1
    elseif is_key_pressed(KeyCode.DOWN)
        keyControls.down = 1
    end
    if is_key_pressed(KeyCode.LEFT)
        keyControls.left = 1
    elseif is_key_pressed(KeyCode.RIGHT)
        keyControls.right = 1
    end
    if is_key_pressed(KeyCode.R)
        keyControls.reset = 1
    end
    return keyControls
end


function mapKeyToCarControl(keys, res, N)
#=
    if keys.up == 1
         res[ 7] = 10
    elseif keys.down == 1
        res[ 7] = -10
    else
        res[7] = 0
    end

    if keys.right == 1
         res[ 8] = -VehicleModel.max_steering_angle
    elseif keys.left == 1
         res[ 8] = VehicleModel.max_steering_angle
    else
        res[8] = 0
    end

    =#
    if keys.reset == 1
        for i in 1:N
            res[8*i + 1] = 0; res[8*i + 2] = 0; res[8*i + 3] = 0.01; res[8*i + 4] = pi/2
        end
    end

    res
end


function initMpcSolver(N, dt, itpTrack, itpLeftBound, itpRightBound, printLevel, max_speed)
    startPose = VehicleModel.CarPose(0,0,0.1,pi/2, 0, 0)
    stateVector = []
    start_=[startPose.x, startPose.y, startPose.x_d, startPose.psi, 0, 0, 0, 0]
    for i in 0:N
        stateVector = vcat(stateVector, start_) #add initial guess to vector
    end
    evalPoints = RaceCourse.getSplinePositions(itpTrack, stateVector, N)
    trackPoints = RaceCourse.getTrackPoints(itpTrack, itpLeftBound, itpRightBound, evalPoints, N)
    forwardPoint = RaceCourse.getForwardTrackPoint(itpTrack, evalPoints, N)

    mpc_struct = MPCStruct(N, 0, 0, 0, 0, 0)
    mpc_struct = init_MPC(mpc_struct, N, dt, startPose, printLevel, max_speed)
    #mpc_struct = define_constraint_nonlinear_bycicle(mpc_struct)
    mpc_struct = define_constraint_linear_bycicle(mpc_struct)
    mpc_struct = define_constraint_start_pose(mpc_struct, startPose)
    mpc_struct = define_constraint_tangents(mpc_struct, trackPoints)
    mpc_struct = define_constraint_max_search_dist(mpc_struct, trackPoints)
    mpc_struct = define_objective(mpc_struct)
    #mpc_struct = define_objective_middle(mpc_struct)
    #mpc_struct = define_objective_minimize_dist(mpc_struct)
    mpc_struct = update_track_forward_point(mpc_struct, forwardPoint)

    return mpc_struct
end

windowSizeX = 1000
windowSizeY = 500
windowSizeMeterX = 100
windowSizeMeterY = 50
scaleX = windowSizeX/windowSizeMeterX
scaleY = windowSizeY/windowSizeMeterY

positionOffsetMeterX = 20
positionOffsetMeterY = 25

radius = 15
trackWidth = 4

keys = KeyControls(0,0,0,0,0)
carPose = VehicleModel.CarPose(0,0,0.1,pi/2, 0, 0)

#define which racecourse should be used
#itpTrack, itpLeftBound, itpRightBound = RaceCourse.buildRaceTrack(15, 4, 15, 0)
#itpTrack, itpLeftBound, itpRightBound = RaceCourse.buildRaceTrack2(trackWidth)
itpTrack, itpLeftBound, itpRightBound = RaceCourse.buildRaceTrack3(trackWidth)

N = 50
printLevel = 0
dt = 0.05
max_speed = 5
mpc_struct = initMpcSolver(N, dt, itpTrack, itpLeftBound, itpRightBound, printLevel, max_speed)
event = Event()
window = RenderWindow("test", windowSizeX, windowSizeY)
#create CircularBuffer for tracking Vehicle Path
carPathBuffer = CircularBuffer{VehicleModel.CarState}(1000)
carPathBufferNonLinear = CircularBuffer{VehicleModel.CarState}(1000)

#create Sprites
RaceTrackLeftSprite, RaceTrackRightSprite = createRaceCourse2(scaleX, scaleY, positionOffsetMeterX, positionOffsetMeterY, itpTrack, itpLeftBound, itpRightBound, window)

set_framerate_limit(window, convert(Int64, 1 / dt))
#set_framerate_limit(window, 2)

clock = Clock()
#lapTimeActive needed for timer
lapTimeActive = false
steps = 0
realCarStateVector = VehicleModel.CarPose(0,0,0.01,pi/2,0,0)
stateVectorNonLinear = VehicleModel.CarPose(0,0,0.1,pi/2, 0, 0)

trackVehicleControls = []
while isopen(window)
    #dt = get_elapsed_time(clock)
    restart(clock)
    while pollevent(window, event)
        if get_type(event) == EventType.CLOSED
            close(window)
        end
    end
    keys = checkkeys()
    res, status = solve_MPC(mpc_struct)
    res = mapKeyToCarControl(keys, res, N)
    #print("\n\nres", res[1:8])
    #stateVector = mapKeyToCarControl(keys, stateVector, N)

    #predict last point and compute next state with vehicle model
    realCarStateVector = VehicleModel.computeCarStepLinearModel(realCarStateVector, res, dt)

    trackVehicleControls = vcat(trackVehicleControls, VehicleModel.CarControls(res[7], res[8]))
    stateVector = VehicleModel.createNewStateVector(res, realCarStateVector, dt, N)
    update_start_point_from_pose(mpc_struct, realCarStateVector)
    evalPoints = RaceCourse.getSplinePositions(itpTrack, stateVector, N)
    trackPoints = RaceCourse.getTrackPoints(itpTrack, itpLeftBound, itpRightBound, evalPoints, N)
    forwardPoint = RaceCourse.getForwardTrackPoint(itpTrack, evalPoints, N)
    update_track_forward_point(mpc_struct, forwardPoint)
    update_track_points(mpc_struct, trackPoints)

    carPose = VehicleModel.CarPose(stateVector[1], stateVector[2], stateVector[3], stateVector[4], stateVector[5], stateVector[6])
    stateVectorNonLinear = VehicleModel.computeCarStepNonLinear(stateVectorNonLinear, res, dt)
    #timer
    steps = steps + 1
    if abs(carPose.x) > 5
        lapTimeActive = true
    end
    if(steps > 50 && RaceCourse.computeDistToTrackStartX(itpTrack, itpLeftBound, realCarStateVector) < trackWidth/2.0)
        if(RaceCourse.computeDistToTrackStartY(realCarStateVector) < 2 && realCarStateVector.y > 0.0)
            println("lap_time_steps:", steps * dt)
            restart(clock)
            steps = 0
            lapTimeActive = false
            break
        end
    end
    #add position to carPathBuffer
    push!(carPathBuffer, VehicleModel.CarState(stateVector[1], stateVector[2], stateVector[3], stateVector[4], stateVector[5], stateVector[6], stateVector[7], stateVector[8]))
    push!(carPathBufferNonLinear, VehicleModel.CarState(stateVectorNonLinear.x, stateVectorNonLinear.y, stateVectorNonLinear.x_d, stateVectorNonLinear.psi, stateVectorNonLinear.y_d, stateVectorNonLinear.psi_d, stateVector[7], stateVector[8]))

    #draw car and raceCource
    carSprite = createcarsprite(scaleX, scaleY)
    setpositioncar(carSprite, stateVectorNonLinear, scaleX, scaleY, positionOffsetMeterX, positionOffsetMeterY)
    drawRaceCourse2(window, RaceTrackLeftSprite, RaceTrackRightSprite)
    # draw tangents for future points
    createTangent(stateVector, itpTrack, itpLeftBound, itpRightBound, scaleX, scaleY, positionOffsetMeterX, positionOffsetMeterY, window)
    #draw carPathBuffer
    createCarPathPoint(carPathBuffer, scaleX, scaleY, positionOffsetMeterX, positionOffsetMeterY, window, SFML.green)
    createCarPathPoint(carPathBufferNonLinear, scaleX, scaleY, positionOffsetMeterX, positionOffsetMeterY, window, SFML.red)
    #draw predicted movement of car
    createPredictionPoints(stateVector, scaleX, scaleY, positionOffsetMeterX, positionOffsetMeterY, window, N)
    draw(window, carSprite)
    #draw car info
    displayCarData(res, window)
    display(window)
    clear(window, SFML.white)
end

stateVectorStart = VehicleModel.CarPose(0,0,0.1,pi/2, 0, 0)
stateVectorLinear = VehicleModel.CarPose(0,0,0.1,pi/2, 0, 0)

stateVectorNonLinear_Base = VehicleModel.CarPose(0,0,0.1,pi/2, 0, 0)
stateVectorNonLinear_Enhanced_Long = VehicleModel.CarPose(0,0,0.1,pi/2, 0, 0)
stateVectorNonLinear_Enhanced_Long_Cmplx = VehicleModel.CarPose(0,0,0.1,pi/2, 0, 0)

stateVectorNonLinear_Enhanced_Lat = VehicleModel.CarPose(0,0,0.1,pi/2, 0, 0)
stateVectorNonLinear_Enhanced_Lat_Cmplx = VehicleModel.CarPose(0,0,0.1,pi/2, 0, 0)


size = length(trackVehicleControls)
dist_lin = []
dist_base = []
dist_long = []
dist_long_cmplx = []
dist_lat = []

for i in 1:size
    carControl = trackVehicleControls[i]
    #throttle = trackVehicleControls[i]
    #phi = trackVehicleControls[i + 1]
    #carControl = VehicleModel.CarControls(throttle, phi)

    stateVectorLinear =  VehicleModel.linear_bycicle_model(stateVectorLinear, carControl, dt)

    stateVectorNonLinear_Base =  VehicleModel.non_linear_model_base(stateVectorNonLinear_Base, carControl, dt)
    stateVectorNonLinear_Enhanced_Long =  VehicleModel.non_linear_model_enhanced_long(stateVectorNonLinear_Enhanced_Long, carControl, dt)
    stateVectorNonLinear_Enhanced_Long_Cmplx =  VehicleModel.non_linear_model_enhanced_long_cmplx(stateVectorNonLinear_Enhanced_Long_Cmplx, carControl, dt)

    stateVectorNonLinear_Enhanced_Lat =  VehicleModel.non_linear_model_enhanced_lat(stateVectorNonLinear_Enhanced_Lat, carControl, dt)
    stateVectorNonLinear_Enhanced_Lat_Cmplx =  VehicleModel.non_linear_model_enhanced_lat_cmplx(stateVectorNonLinear_Enhanced_Lat_Cmplx, carControl, dt)

    dist_tmp = sqrt((stateVectorNonLinear_Enhanced_Lat_Cmplx.x - stateVectorLinear.x)^2 + (stateVectorNonLinear_Enhanced_Lat_Cmplx.y - stateVectorLinear.y)^2)
    dist_lin = vcat(dist_lin, dist_tmp)

    dist_tmp = sqrt((stateVectorNonLinear_Enhanced_Lat_Cmplx.x - stateVectorNonLinear_Base.x)^2 + (stateVectorNonLinear_Enhanced_Lat_Cmplx.y - stateVectorNonLinear_Base.y)^2)
    dist_base = vcat(dist_base, dist_tmp)

    dist_tmp = sqrt((stateVectorNonLinear_Enhanced_Lat_Cmplx.x - stateVectorNonLinear_Enhanced_Long.x)^2 + (stateVectorNonLinear_Enhanced_Lat_Cmplx.y - stateVectorNonLinear_Enhanced_Long.y)^2)
    dist_long = vcat(dist_long, dist_tmp)

    dist_tmp = sqrt((stateVectorNonLinear_Enhanced_Lat_Cmplx.x - stateVectorNonLinear_Enhanced_Long_Cmplx.x)^2 + (stateVectorNonLinear_Enhanced_Lat_Cmplx.y - stateVectorNonLinear_Enhanced_Long_Cmplx.y)^2)
    dist_long_cmplx = vcat(dist_long_cmplx, dist_tmp)

    dist_tmp = sqrt((stateVectorNonLinear_Enhanced_Lat_Cmplx.x - stateVectorNonLinear_Enhanced_Lat.x)^2 + (stateVectorNonLinear_Enhanced_Lat_Cmplx.y - stateVectorNonLinear_Enhanced_Lat.y)^2)
    dist_lat = vcat(dist_lat, dist_tmp)

end
x = linspace(0, 1, size)
plot(x, dist_lin, linewidth=1.0, linestyle="--", label=L"dist_lin$")
plot(x, dist_base, linewidth=1.0, linestyle="--", label=L"dist_base$")
plot(x, dist_long, linewidth=1.0, linestyle="--", label=L"dist_long$")
plot(x, dist_long_cmplx, linewidth=1.0, linestyle="--", label=L"dist_long_cmplx$")
plot(x, dist_lat, linewidth=1.0, linestyle="--", label=L"dist_lat$")
ax = gca()
ax[:legend](loc="upper left")
xlabel("Distance on Track")
ylabel("Difference to Most Accurate Model")
title("Difference Basic to Complex Model")

#print("\nstateVectorLinear", stateVectorLinear)
#print("\nstateVectorNonLinear_Base", stateVectorNonLinear_Base)
#print("\nstateVectorNonLinear_Enhanced_Long", stateVectorNonLinear_Enhanced_Long)
#print("\nstateVectorNonLinear_Enhanced_Long_Cmplx", stateVectorNonLinear_Enhanced_Long_Cmplx)
#print("\nstateVectorNonLinear_Enhanced_Lat", stateVectorNonLinear_Enhanced_Lat)
#print("\nstateVectorNonLinear_Enhanced_Lat_Cmplx", stateVectorNonLinear_Enhanced_Lat_Cmplx)
