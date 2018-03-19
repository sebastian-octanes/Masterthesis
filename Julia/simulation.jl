using SFML


global acceleration = 0;
left = 0;
right = 0;
reset = 0;
global x_position = 0;
global y_position = 0;


function createcarsprite()
    texture = Texture("/home/weller/Master/Julia/race_car.jpg")
    set_smooth(texture, true)
    global carsprite = Sprite()
    set_texture(carsprite, texture)
    scale(carsprite, Vector2f(0.1, 0.1))
    return carsprite
end

function rotatecar(sprite, degree)
    rotate(carsprite, degree)
end

function setpositioncar(sprite, pos)
    set_position(carsprite, pos)
end

function checkkeys()
    if is_key_pressed(KeyCode.UP)
        global acceleration = 1
    elseif is_key_pressed(KeyCode.DOWN)
        global acceleration = -1
    end
    if is_key_pressed(KeyCode.LEFT)
        global left = 1
    elseif is_key_pressed(KeyCode.RIGHT)
        global right = 1
    end
    if is_key_pressed(KeyCode.R)
        global reset = 1
    end
end

function updatevehicleposition()
    println("called updatevehiclepos", acceleration)
    println("vehicle pose", x_position)
    if acceleration == 1
        global x_position = x_position + 1
        global acceleration = 0
    end

    carpose = Vector2f(x_position, y_position)
    setpositioncar(carsprite, carpose)
end

window = RenderWindow("test",800,600)
set_framerate_limit(window, 60)

event = Event()

carsprite = createcarsprite()
carposition = Vector2f(0.0, 0.0)
setpositioncar(carsprite, carposition)

while isopen(window)
    while pollevent(window, event)
        if get_type(event) == EventType.CLOSED
            close(window)
        end
    end
    checkkeys()
    updatevehicleposition()

    clear(window, SFML.white)
    draw(window, carsprite)
    display(window)
end
