using SFML
include("RaceCourse.jl")
include("VehicleModel.jl")
include("MPC.jl")
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
    set_origin(rect, Vector2f(carSizeX/2, carSizeY/2))
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

function createPredictionPoints(res, scaleX, scaleY, offsetX, offsetY, window, N)
    println("res[1]", res[1])
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

function createTangent(carPose, itpTrack, itpOutBound, itpInBound, scaleX, scaleY, offsetX, offsetY, window)

    i = RaceCourse.getSplinePosition(itpTrack, carPose.x, carPose.y)
    x =  RaceCourse.computeGradientPoints(itpOutBound, i)
    alpha = RaceCourse.computeGradientAngle(itpOutBound, i)
    line = RectangleShape()
    set_size(line, Vector2f(100, 1))
    set_outline_thickness(line, 1)
    rotate(line, - alpha *180/pi)
    set_fillcolor(line, SFML.green)
    set_origin(line, Vector2f(50, 1.5))
    off = Vector2f(offsetX * scaleX + x[1] * scaleX , offsetY * scaleY - x[2] * scaleY )
    set_position(line, off)
    draw(window, line)

    x =  RaceCourse.computeGradientPoints(itpInBound, i)
    alpha = RaceCourse.computeGradientAngle(itpInBound, i)
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


function initMpcSolver(N, dt, itpTrack, itpOutBound, itpInBound)
    startPose = VehicleModel.CarPose(0,0,0.01,pi/2)
    stateVector = []
    #global itpTrack, itpOutBound, itpInBound = RaceCourse.buildRaceTrack(15, 4, 15, 0)
    start_=[startPose.x, startPose.y, startPose.v, startPose.yaw, 0, 0]
    for i in 0:N
        stateVector = vcat(stateVector, start_) #add initial guess to vector
    end
    evalPoints = RaceCourse.getSplinePositions(itpTrack, stateVector, N)
    tangentPoints = RaceCourse.computeGradientPoints_(itpOutBound, itpInBound, evalPoints, N)
    m = MPC.initMPC(N, dt, startPose, tangentPoints, 0)
    return m
end

windowSizeX = 1000
windowSizeY = 500
windowSizeMeterX = 100
windowSizeMeterY = 50
scaleX = windowSizeX/windowSizeMeterX
scaleY = windowSizeY/windowSizeMeterY

positionOffsetMeterX = 50
positionOffsetMeterY = 25

radius = 15


keys = KeyControls(0,0,0,0,0)
carPose = VehicleModel.CarPose(0,0,0,pi/2)
itpTrack, itpOutBound, itpInBound = RaceCourse.buildRaceTrack(15, 4, 15, 0)

N = 45

dt = 0.05
initMpcSolver(N, dt, itpTrack, itpOutBound, itpInBound)
event = Event()
window = RenderWindow("test", windowSizeX, windowSizeY)
set_framerate_limit(window, convert(Int64, 1 / dt))
clock = Clock()
while isopen(window)
    dt = get_elapsed_time(clock)
    restart(clock)
    while pollevent(window, event)
        if get_type(event) == EventType.CLOSED
            close(window)
        end
    end
    keys = checkkeys()

    res = MPC.solveMPC()
    res = mapKeyToCarControl(keys, res, N)
    MPC.updateStartPoint(res)
    evalPoints = RaceCourse.getSplinePositions(itpTrack, res, N)
    tangentPoints = RaceCourse.computeGradientPoints_(itpOutBound, itpInBound, evalPoints, N)
    MPC.updateTangentPoints(tangentPoints)
    carPose = VehicleModel.CarPose(res[1], res[2], res[3], res[4])

    #draw car and raceCource
    carSprite = createcarsprite(scaleX, scaleY)
    setpositioncar(carSprite, carPose, scaleX, scaleY, positionOffsetMeterX, positionOffsetMeterY)
    trackOut = createRaceCourse(scaleX, scaleY, radius +2, positionOffsetMeterX, positionOffsetMeterY)
    track = createRaceCourse(scaleX, scaleY, radius, positionOffsetMeterX, positionOffsetMeterY)
    trackIn = createRaceCourse(scaleX, scaleY, radius -2, positionOffsetMeterX, positionOffsetMeterY)
    draw(window, trackOut)
    #draw(window, track)
    draw(window, trackIn)
    for i in 1:3:N
        carPose = VehicleModel.CarPose(res[6*i + 1], res[6*i + 2], res[6*i + 3], res[6*i + 4])
        createTangent(carPose, itpTrack, itpOutBound, itpInBound, scaleX, scaleY, positionOffsetMeterX, positionOffsetMeterY, window)
    end
    createPredictionPoints(res, scaleX, scaleY, positionOffsetMeterX, positionOffsetMeterY, window, N)
    draw(window, carSprite)
    display(window)
    clear(window, SFML.white)
end
