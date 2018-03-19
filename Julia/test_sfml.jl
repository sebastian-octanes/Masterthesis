using SFML

window = RenderWindow("test",800,600)
set_framerate_limit(window, 60)

event = Event()

text = RenderText()
#texture = Texture(joinpath(dirname(@__FILE__),"..","..","assets","sfmljl_logo.png"))
texture = Texture("/home/weller/Master/Julia/race_car.jpg")
sprite = Sprite()
set_color(sprite, Color(0,255,0))
set_texture(sprite, texture)
set_position(text, Vector2f(400.0, 300.0))
set_string(text, "SFML.jl is cool")
set_style(text, TextBold | TextUnderlined | TextItalic | TextStrikeThrough)
set_color(text, SFML.cyan)
set_charactersize(text, 50)
text_size = get_globalbounds(text)
set_origin(text, Vector2f(text_size.width / 2, text_size.height / 2))

println(get_string(text))
i = 0
while isopen(window)
    while pollevent(window, event)
        if get_type(event) == EventType.CLOSED
            close(window)
        end
    end

    clear(window, SFML.black)
    draw(window, text)
    draw(window, sprite)
    display(window)
end
