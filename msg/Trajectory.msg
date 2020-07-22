# This is a NxM matrix which is reshaped into a NMx1 vector.
# N is the number of time steps, M is the number of features.
# In this trajectory, stride = M and data.size() / stride = N.
uint32    stride
float32[] data
