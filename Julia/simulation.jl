using SFML

function createcarsprite()
    texture = Texture("/home/weller/Master/Julia/race_car.jpg")
    set_smooth(texture, true)
    carsprite = Sprite()
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


window = RenderWindow("test",800,600)
set_framerate_limit(window, 60)

event = Event()

carsprite = createcarsprite()


while isopen(window)

    while pollevent(window, event)
        if get_type(event) == EventType.CLOSED
            close(window)
        end
    end
    text = RenderText()
    set_color(text, SFML.cyan)
    set_charactersize(text, 50)
    set_position(text, Vector2f(400.0, 300.0))
    set_string(text, "up not pressed")

    if is_key_pressed(KeyCode.UP)
        println("test")
        set_string(text, "up key pressed")
    end

    clear(window, SFML.white)
    draw(window, text)
    draw(window, carsprite)
    display(window)
end
