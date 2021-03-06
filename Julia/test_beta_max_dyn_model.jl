using SFML
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
function createCarPathPoint(carPathBuffer, scaleX, scaleY, offsetX, offsetY, window)
    circle = CircleShape()
    set_radius(circle, 1)
    set_origin(circle, Vector2f(1, 1))
    for i in 1:length(carPathBuffer)
        carState = carPathBuffer[i]
        if carState.throttle > 0
            green = round(Int16, 25 * carState.throttle * 1.8)
            if green > 255
                green = 255
            end
            set_fillcolor(circle, Color(0, green , 0 ))
        else
            red =  round(Int16, 25 *-carState.throttle * 1.8)
            if red > 255
                red = 255
            end
            set_fillcolor(circle, Color( red, 0 , 0))
        end
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

    if keys.reset == 1
        for i in 1:N
            res[8*i + 1] = 0; res[8*i + 2] = 0; res[8*i + 3] = 0.01; res[8*i + 4] = pi/2
        end
    end

    res
end


function initMpcSolver(N, dt, startPose, itpTrack, itpLeftBound, itpRightBound, printLevel, max_speed, trackWidth, beta, a, b)
    stateVector = []
    start_=[startPose.x, startPose.y, startPose.x_d, startPose.psi, 0, 0, 0, 0]
    for i in 0:N
        stateVector = vcat(stateVector, start_) #add initial guess to vector
    end
    evalPoints = RaceCourse.getSplinePositions(itpTrack, stateVector, N)
    trackPoints = RaceCourse.getTrackPoints(itpTrack, itpLeftBound, itpRightBound, evalPoints, N)
    forwardPoint = RaceCourse.getForwardTrackPoint(itpTrack, evalPoints, N)
    forwardPointBounds = RaceCourse.getForwardTrackPointBounds(itpLeftBound, itpRightBound, evalPoints, N)

    print("last TrackPoint", trackPoints)
    print("\nforwardPoint", forwardPoint)
    mpc_struct = MPCStruct(N, 0, 0, 0, 0, 0, 0)
    mpc_struct = init_MPC(mpc_struct, N, dt, startPose, printLevel, max_speed, trackWidth, beta)
    #mpc_struct = define_constraint_nonlinear_bycicle(mpc_struct)
    mpc_struct = define_constraint_kin_bycicle(mpc_struct)
    mpc_struct = define_constraint_start_pose(mpc_struct, startPose)
    #mpc_struct = define_constraint_tangents(mpc_struct, trackPoints)
    #mpc_struct = define_constraint_max_search_dist(mpc_struct, trackPoints)
    #mpc_struct = define_objective(mpc_struct)
    #mpc_struct = define_objective_middle(mpc_struct)
    #mpc_struct = define_objective_minimize_dist(mpc_struct)
    #mpc_struct = define_objective_max_track_dist(mpc_struct)
    #mpc_struct = define_objective_minimize_dist_soft_const(mpc_struct,2, 1)
    mpc_struct = define_objective_minimize_dist_soft_const_ext(mpc_struct,a, b)

    #mpc_struct = update_track_forward_point(mpc_struct, forwardPoint)
    mpc_struct = update_track_forward_point_bounds(mpc_struct, forwardPoint, forwardPointBounds)
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
startPose = VehicleModel.CarPose(0,0,VehicleModel.min_speed, pi/2, 0, 0)

#define which racecourse should be used
#itpTrack, itpLeftBound, itpRightBound = RaceCourse.buildRaceTrack(15, 4, 15, 0)
#itpTrack, itpLeftBound, itpRightBound = RaceCourse.buildRaceTrack2(trackWidth)
#itpTrack, itpLeftBound, itpRightBound = RaceCourse.buildRaceTrack3(trackWidth)
itpTrack, itpLeftBound, itpRightBound = RaceCourse.buildRaceTrack4(trackWidth)
#itpTrack, itpLeftBound, itpRightBound = RaceCourse.buildRaceTrackCircle(trackWidth)
#itpTrack, itpLeftBound, itpRightBound = RaceCourse.buildRaceTrackStraight(trackWidth)



printLevel = 0
dt = 0.05
max_speed = 30

event = Event()
window = RenderWindow("test", windowSizeX, windowSizeY)

#create Sprites
RaceTrackLeftSprite, RaceTrackRightSprite = createRaceCourse2(scaleX, scaleY, positionOffsetMeterX, positionOffsetMeterY, itpTrack, itpLeftBound, itpRightBound, window)

set_framerate_limit(window, convert(Int64, 1 / dt))


#create CircularBuffer for tracking Vehicle Path
carPathBuffer = CircularBuffer{VehicleModel.CarState}(400)
beta = 1.2:0.1:1.2
#n = [27,28,29,30,31,32,33,34]
n = [21]

lap_time1 = []
lap_time2 = []
n_out = []
beta_out = []
lap_time_out1 = []
lap_time_out2 = []
count = 0
for N in n
    for b in beta
        percent = count/(size(n)[1]* size(beta)[1])
        println("percent: ",percent)
        count = count + 1
        n_out = vcat(n_out, N)
        beta_out = vcat(beta_out, b)
        #set_framerate_limit(window, 2)
        mpc_struct = initMpcSolver(N, dt, startPose, itpTrack, itpLeftBound, itpRightBound, printLevel, max_speed, trackWidth, b * 9.81, 1, 1)
        laps = 1
        clock = Clock()
        #lapTimeActive needed for timer
        lapTimeActive = false
        steps = 0
        realCarStateVector = VehicleModel.CarPose(0,0,VehicleModel.min_speed, pi/2,0,0)
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
            #realCarStateVector = VehicleModel.computeCarStepDynFinal(realCarStateVector, res, dt)
            #realCarStateVector = VehicleModel.computeCarStepDynModelBase(realCarStateVector, res, dt)
            #realCarStateVector = VehicleModel.computeCarStepDynModelLatLin(realCarStateVector, res, dt)
            #realCarStateVector = VehicleModel.computeCarStepDynModelLatSimple(realCarStateVector, res, dt)
            realCarStateVector = VehicleModel.computeCarStepDynKamsch(realCarStateVector, res, dt)


            stateVector = VehicleModel.createNewStateVector(res, realCarStateVector, dt, N)
            update_start_point_from_pose(mpc_struct, realCarStateVector)
            evalPoints = RaceCourse.getSplinePositions(itpTrack, stateVector, N)
            trackPoints = RaceCourse.getTrackPoints(itpTrack, itpLeftBound, itpRightBound, evalPoints, N)
            forwardPoint = RaceCourse.getForwardTrackPoint(itpTrack, evalPoints, N)
            #print("\n position: ", realCarStateVector)
            #print("\nforward Point", forwardPoint)
            update_track_forward_point(mpc_struct, forwardPoint)
            update_track_points(mpc_struct, trackPoints)

            #timer
            steps = steps + 1
            if abs(realCarStateVector.x) > 5
                lapTimeActive = true
            end

            if(abs(RaceCourse.computeDistToTrackBorder(itpTrack, itpLeftBound, realCarStateVector)) > (trackWidth/2.0 + 0.2))
                println("outside of course", RaceCourse.computeDistToTrackBorder(itpTrack, itpLeftBound, realCarStateVector))
                if(laps == 1)
                    lap_time_out1 = vcat(lap_time_out1, 0.0)
                    lap_time_out2 = vcat(lap_time_out2, 0.0)
                elseif(laps == 2)
                    lap_time_out2 = vcat(lap_time_out2, 0.0)
                end
                break;
            end
            #if abs(realCarStateVector.x) < trackWidth/2 && abs(realCarStateVector.y) < 0.4 && lapTimeActive
            if(steps > 50 && RaceCourse.computeDistToTrackStartX(itpTrack, itpLeftBound, realCarStateVector) < trackWidth/2.0)
                if(RaceCourse.computeDistToTrackStartY(realCarStateVector) < 2 && realCarStateVector.y > 0.0)
                    println("\nlap_time_steps:", steps * dt)
                    restart(clock)

                    if(laps == 1)
                        lap_time_out1 = vcat(lap_time_out1, steps * dt)
                    elseif(laps == 2)
                        lap_time_out2 = vcat(lap_time_out2, steps * dt)
                    end
                    laps = laps + 1
                    steps = 0
                    lapTimeActive = false

                    if(laps >= 3)
                        break
                    end
                end
            end
            #add position to carPathBuffer
            push!(carPathBuffer, VehicleModel.CarState(stateVector[1], stateVector[2], stateVector[3], stateVector[4], stateVector[5], stateVector[6], stateVector[7], stateVector[8]))

            #draw car and raceCource
            carSprite = createcarsprite(scaleX, scaleY)
            setpositioncar(carSprite, realCarStateVector, scaleX, scaleY, positionOffsetMeterX, positionOffsetMeterY)
            drawRaceCourse2(window, RaceTrackLeftSprite, RaceTrackRightSprite)
            # draw tangents for future points
            #createTangent(stateVector, itpTrack, itpLeftBound, itpRightBound, scaleX, scaleY, positionOffsetMeterX, positionOffsetMeterY, window)
            #draw carPathBuffer
            createCarPathPoint(carPathBuffer, scaleX, scaleY, positionOffsetMeterX, positionOffsetMeterY, window)
            #draw predicted movement of car
            createPredictionPoints(stateVector, scaleX, scaleY, positionOffsetMeterX, positionOffsetMeterY, window, N)
            draw(window, carSprite)
            #draw car info
            displayCarData(res, window)
            display(window)
            clear(window, SFML.white)
        end
    end
end

println("n_out", n_out)
println("beta_out", beta_out)
println("lap_time_out1", lap_time_out1)
println("lap_time_out2", lap_time_out2)


open("outputFiles/testBetaMaxFine.dat", "w") do io
    writedlm(io, [n_out beta_out lap_time_out1 lap_time_out2])
end
