include("RaceCourse.jl")
using Interpolations
using PyPlot

test = [1,2,3,4,5,6,7,8,9,10]

test = circshift(test, (-2))

println(test)
