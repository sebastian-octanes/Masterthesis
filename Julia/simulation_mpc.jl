using SFML
include("RaceCourse.jl")
include("VehicleModel.jl")
include("MPC.jl")
include("circular_buffer.jl")
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
        if carState.acc > 0
            green = round(Int16, 25 * carState.acc * 1.8)
            if green > 255
                green = 255
            end
            set_fillcolor(circle, Color(0, green , 0 ))
        else
            red =  round(Int16, 25 *-carState.acc * 1.8)
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
        x = res[i*6 + 1]
        y = - res[i*6 + 2]
        set_position(circle, Vector2f(offsetX*scaleX + x*scaleX, offsetY*scaleY + y*scaleY))
        draw(window, circle)
    end
end

function createTangent(carPose, itpTrack, itpLeftBound, itpRightBound, scaleX, scaleY, offsetX, offsetY, window)

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

function displayCarData(res, window)
    #text = TextRegular()
    text = RenderText()
    speed = res[3] *3.61
    set_string(text, "speed $(speed) km/h")
    set_color(text, SFML.red)
    set_charactersize(text, 25)
    set_position(text, Vector2f(50,450))
    draw(window, text)
end

function setpositioncar(carsprite, carPose, scaleX, scaleY, offsetX, offsetY)
    set_position(carsprite, Vector2f(scaleX * carPose.x + offsetX*scaleX, (- scaleY *carPose.y + offsetY*scaleY)))
    rotate(carsprite, -(carPose.yaw )*180/pi)
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
            res[6*i + 1] = 0; res[6*i + 2] = 0; res[6*i + 3] = 0.01; res[6*i + 4] = pi/2
        end
    end
    res
end


function initMpcSolver(N, dt, itpTrack, itpLeftBound, itpRightBound, printLevel)
    startPose = VehicleModel.CarPose(0,0,0.1,pi/2)
    stateVector = []
    #global itpTrack, itpLeftBound, itpRightBound = RaceCourse.buildRaceTrack(15, 4, 15, 0)
    start_=[startPose.x, startPose.y, startPose.v, startPose.yaw, 0, 0]
    for i in 0:N
        stateVector = vcat(stateVector, start_) #add initial guess to vector
    end
    evalPoints = RaceCourse.getSplinePositions(itpTrack, stateVector, N)
    tangentPoints = RaceCourse.computeGradientPoints_(itpLeftBound, itpRightBound, evalPoints, N)
    midTrackPoints = RaceCourse.getMidTrackPoints(itpTrack,evalPoints, N)
    trackPoints = RaceCourse.getTrackPoints(itpTrack, itpLeftBound, itpRightBound, evalPoints, N)
    println(midTrackPoints)
    m = MPC.initMPC(N, dt, startPose, tangentPoints, midTrackPoints, trackPoints, printLevel)
    return m
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
carPose = VehicleModel.CarPose(0,0,0.1,pi/2)
#itpTrack, itpLeftBound, itpRightBound = RaceCourse.buildRaceTrack(15, 4, 15, 0)
itpTrack, itpLeftBound, itpRightBound = RaceCourse.buildRaceTrack2(trackWidth)

N = 20
printLevel = 0
dt = 0.05
initMpcSolver(N, dt, itpTrack, itpLeftBound, itpRightBound, printLevel)
event = Event()
window = RenderWindow("test", windowSizeX, windowSizeY)
#create CircularBuffer for tracking Vehicle Path
#carPathBuffer = CircBuffer.CircularBuffer{VehicleModel.CarState}(400)
carPathBuffer = CircularBuffer{VehicleModel.CarState}(400)

#create Sprites
RaceTrackLeftSprite, RaceTrackRightSprite = createRaceCourse2(scaleX, scaleY, positionOffsetMeterX, positionOffsetMeterY, itpTrack, itpLeftBound, itpRightBound, window)

set_framerate_limit(window, convert(Int64, 1 / dt))
#set_framerate_limit(window, 5)

clock = Clock()
#lapTimeActive needed for timer
lapTimeActive = false
steps = 0
while isopen(window)
    #dt = get_elapsed_time(clock)
    restart(clock)
    while pollevent(window, event)
        if get_type(event) == EventType.CLOSED
            close(window)
        end
    end
    keys = checkkeys()

    @time res = MPC.solveMPC()
    res = mapKeyToCarControl(keys, res, N)
    stateVector = VehicleModel.createNewStateVector(res, dt, N)
    MPC.updateStartPoint(stateVector)
    evalPoints = RaceCourse.getSplinePositions(itpTrack, stateVector, N)
    tangentPoints = RaceCourse.computeGradientPoints_(itpLeftBound, itpRightBound, evalPoints, N)
    midTrackPoints = RaceCourse.getMidTrackPoints(itpTrack, evalPoints, N)
    trackPoints = RaceCourse.getTrackPoints(itpTrack, itpLeftBound, itpRightBound, evalPoints, N)
    MPC.updateTangentPoints(tangentPoints)
    MPC.updateMidTrackPoints(midTrackPoints)
    MPC.updateTrackPoints(trackPoints)
    carPose = VehicleModel.CarPose(stateVector[1], stateVector[2], stateVector[3], stateVector[4])
    #timer
    steps = steps + 1
    if abs(carPose.x) > 5
        lapTimeActive = true
    end
    if abs(carPose.x) < trackWidth/2 && abs(carPose.y) < 0.4 && lapTimeActive
        println("lap_time_steps:", steps * dt)
        restart(clock)
        steps = 0
        lapTimeActive = false
    end

    #add position to carPathBuffer
    push!(carPathBuffer, VehicleModel.CarState(stateVector[1], stateVector[2], stateVector[3], stateVector[4], stateVector[5], stateVector[6]))

    #draw car and raceCource
    carSprite = createcarsprite(scaleX, scaleY)
    setpositioncar(carSprite, carPose, scaleX, scaleY, positionOffsetMeterX, positionOffsetMeterY)
    drawRaceCourse2(window, RaceTrackLeftSprite, RaceTrackRightSprite)
    # draw tangents for future points
    for i in 1:6:N
        carPose = VehicleModel.CarPose(stateVector[6*i + 1], stateVector[6*i + 2], stateVector[6*i + 3], stateVector[6*i + 4])
        createTangent(carPose, itpTrack, itpLeftBound, itpRightBound, scaleX, scaleY, positionOffsetMeterX, positionOffsetMeterY, window)
    end
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
