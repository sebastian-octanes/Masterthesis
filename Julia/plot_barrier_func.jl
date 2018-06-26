linear = []
quadratic = []
div = []
euler= []

max_steps = 20
lin = linspace(-0, 5, max_steps)

m1 = 0
m2 = 0.1
m3 = 1.0
m4 = 2.0
m5 = 5.0
m6 = 10.0

mu1 = []
mu2 = []
mu3 = []
mu4 = []
mu5 = []
mu6 = []

print("lin", lin)

for i in lin
       mu1 = vcat(mu1, -m1 * log(i))
       mu2 = vcat(mu2, -m2 * log(i))
       mu3 = vcat(mu3, -m3 * log(i))
       mu4 = vcat(mu4, -m4 * log(i))
       mu5 = vcat(mu5, -m5 * log(i))
       mu6 = vcat(mu6, -m6 * log(i))
end

open("outputFiles/barrierFunc.dat", "w") do io
    writedlm(io, [lin  mu1 mu2 mu3 mu4 mu5 mu6])
end
