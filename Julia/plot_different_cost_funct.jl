using PyPlot


function cost_lin(x)
       alpha = 10.0
       ret =  alpha * abs(x)
       return ret
end

function cost_quad(x)
   #alpha = 2/(8*(self.track.track_width/8.0)**7)
   alpha = 5
   return alpha *(x)^2
end


function cost_div( x)
       alpha = 1
       k1 = 2.5
       k2 = -2.5
       return abs(alpha/(k1 - x) + alpha/(k2 - x))
end


function cost_euler(x)
       alpha = 10
       k1 = -2.5
       k2 = 2.5
       return e^(alpha*(k1 + x)) + e^(-alpha*(k2 + x))
end






linear = []
quadratic = []
div = []
euler= []

max_steps = 100
lin = linspace(-3, 3, max_steps)
print(lin)
for i in lin
       linear = vcat(linear, cost_lin(i))
       quadratic = vcat(quadratic, cost_quad(i))
       div = vcat(div, cost_div(i))
       euler = vcat(euler, cost_euler(i))

end
print(linear)



open("outputFiles/costFunct.txt", "w") do io
    writedlm(io, [lin linear quadratic  div euler])
end
