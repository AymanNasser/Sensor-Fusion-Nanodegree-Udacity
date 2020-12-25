
def update(mean1, var1, mean2, var2):
    new_mean = (var2 * mean1 + var1 * mean2) / (var1 + var2)
    new_var = 1/(1/var1 + 1/var2)
    return [new_mean, new_var]

def predict(mean1, var1, mean2, var2):
    new_mean = mean1 + mean2
    new_var = var1 + var2
    return [new_mean, new_var]


measurements = [5., 6., 7., 9., 10.]
motion = [1., 1., 2., 1., 1.]
measurement_sigma = 4. # assume constant variance
motion_sigma = 2. # # assume constant variance
# init position distribution
mu = 0. 
sig = 1000.

for i in range(len(measurements)):

	[mu, sig] = update(mu, sig, measurements[i], measurement_sigma)
	print("Update: ",mu,sig)
	[mu, sig] = predict(mu, sig, motion[i], motion_sigma)
	print("Predict: ", mu, sig)
