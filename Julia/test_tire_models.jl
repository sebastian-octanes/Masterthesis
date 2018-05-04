
using PyPlot
include("RaceCourse.jl")
include("VehicleModel.jl")
include("MPC.jl")
include("circular_buffer.jl")
include("smallMPC.jl")




lin = linspace(-1, 1, 101)
y1= []
y2= []
y3= []

for i in lin
    y1 = vcat(y1, VehicleModel.pacejka_tire_model_linear(i, true))
    y2 = vcat(y2, VehicleModel.pacejka_tire_model(i, true))
    y3 = vcat(y3, VehicleModel.pacejka_tire_model_complex(i, true))
end


plot(lin,y1)
plot(lin,y2)
plot(lin,y3)