using SFML
include("RaceCourse.jl")
include("VehicleModel.jl")
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

function createRaceCourse2(scaleX, scaleY, offsetX, offsetY, itpTrack, itpLeftBound, itpRightBound, window)
    shape = ConvexShape()
    shapeLeft = ConvexShape()
    shapeRight = ConvexShape()

    N = 400
    set_pointcount(shape, N)
    set_outline_thickness(shape, 2)
    set_outlinecolor(shape, SFML.red)
    set_pointcount(shapeLeft, N)
    set_outline_thickness(shapeLeft, 1)
    set_outlinecolor(shapeLeft, SFML.black)
    set_pointcount(shapeRight, N)
    set_outline_thickness(shapeRight, 1)
    set_outlinecolor(shapeRight, SFML.black)

    for i in 0:(N-1)
        x = itpTrack[1/N * i, 1]
        y = itpTrack[1/N * i, 2]
        set_point(shape, i, Vector2f(offsetX*scaleX + x*scaleX , offsetY*scaleY - scaleY *y))
        x = itpLeftBound[1/N * i, 1]
        y = itpLeftBound[1/N * i, 2]
        set_point(shapeLeft, i, Vector2f(offsetX*scaleX + x*scaleX , offsetY*scaleY - scaleY *y))
        x = itpRightBound[1/N * i, 1]
        y = itpRightBound[1/N * i, 2]
        set_point(shapeRight, i, Vector2f(offsetX*scaleX + x*scaleX , offsetY*scaleY - scaleY *y))
        #set_points
    end
    draw(window, shapeLeft)
    draw(window, shape)
    draw(window, shapeRight)
end

function createTangent(carPose, itpTrack, itpOutBound, itpInBound, scaleX, scaleY, offsetX, offsetY, window)

    i = RaceCourse.getSplinePosition(itpTrack, carPose.x, carPose.y)
    x =  RaceCourse.computeGradientPoints(itpOutBound, i)
    alpha = RaceCourse.computeGradientAngle(itpOutBound, i)
    line = RectangleShape()
    set_size(line, Vector2f(100, 3))
    set_outline_thickness(line, 2)
    rotate(line, - alpha *180/pi)
    set_fillcolor(line, SFML.green)
    set_origin(line, Vector2f(50, 1.5))
    off = Vector2f(offsetX * scaleX + x[1] * scaleX , offsetY * scaleY - x[2] * scaleY )
    set_position(line, off)
    draw(window, line)

    x =  RaceCourse.computeGradientPoints(itpInBound, i)
    alpha = RaceCourse.computeGradientAngle(itpInBound, i)
    line = RectangleShape()
    set_size(line, Vector2f(100, 3))
    set_outline_thickness(line, 2)
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

function updatevehicleposition(keyControls, carPose)
    x = carPose.x
    if keyControls.up == 1
        x = carPose.x + 1
    end
    if keyControls.down == 1
        x = carPose.x - 1
    end
    VehicleModel.CarPose(x,0,0,0)
end

function mapKeyToCarControl(keys, carPose)
    acc = 0
    steer = 0
    if keys.up == 1
        acc = VehicleModel.max_long_acc
    elseif keys.down == 1
        acc = - VehicleModel.max_long_dec
    end
    if keys.left == 1
        steer = VehicleModel.max_steering_angle
    elseif keys.right == 1
        steer = - VehicleModel.max_steering_angle
    end
    if keys.reset == 1
        carPose = VehicleModel.CarPose(0,0,0,pi/2)
    end
    carPose, VehicleModel.CarControls(acc, steer)
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
#itpTrack, itpOutBound, itpInBound = RaceCourse.buildRaceTrack(15, 4, 15, 0)
itpTrack, itpLeftBound, itpRightBound = RaceCourse.buildRaceTrack2(4)



event = Event()
window = RenderWindow("test", windowSizeX, windowSizeY)
set_framerate_limit(window, 30)
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
    carPose, carControls = mapKeyToCarControl(keys, carPose)
    carPose = VehicleModel.computeTimeStep(carPose, carControls, as_seconds(dt))


    #draw car and raceCource

    carSprite = createcarsprite(scaleX, scaleY)
    setpositioncar(carSprite, carPose, scaleX, scaleY, positionOffsetMeterX, positionOffsetMeterY)
    trackOut = createRaceCourse(scaleX, scaleY, radius +2, positionOffsetMeterX, positionOffsetMeterY)
    track = createRaceCourse(scaleX, scaleY, radius, positionOffsetMeterX, positionOffsetMeterY)
    trackIn = createRaceCourse(scaleX, scaleY, radius -2, positionOffsetMeterX, positionOffsetMeterY)
    #draw(window, trackOut)
    #draw(window, track)
    #draw(window, trackIn)
    createRaceCourse2(scaleX, scaleY, positionOffsetMeterX, positionOffsetMeterY, itpTrack, itpLeftBound, itpRightBound, window)
    draw(window, carSprite)
    createTangent(carPose, itpTrack, itpLeftBound, itpRightBound, scaleX, scaleY, positionOffsetMeterX, positionOffsetMeterY, window)
    display(window)
    clear(window, SFML.white)
end
