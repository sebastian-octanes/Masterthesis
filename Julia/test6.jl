using SFML

windowSizeX = 1000
windowSizeY = 500


event = Event()

window = RenderWindow("test", windowSizeX, windowSizeY)
set_framerate_limit(window, 4)
clock = Clock()

while isopen(window)
    dt = get_elapsed_time(clock)
    restart(clock)
    while pollevent(window, event)
        if get_type(event) == EventType.CLOSED
            close(window)
        end
    end

    clear(window, SFML.black)

    radius = 30
    offsetX = 100
    offsetY = 50
    circle = CircleShape()
    set_radius(circle, radius)
    set_fillcolor(circle, SFML.red)
    set_position(circle, Vector2f(offsetX, offsetY))
    set_origin(circle, Vector2f(radius, radius))
    set_outline_thickness(circle, 5)
    set_outlinecolor(circle, SFML.green)
    draw(window, circle)
    display(window)
end
