from Brequet_Range_Equation import Mfuellst, addedweightlst, Mfuel_additional, n

# bx.plot(addedweightlst[1:n], Mfuellst[0, 0] - Mfuellst[0, 1:n] - Mfuel_additional, \
#         addedweightlst[1:n], Mfuellst[1, 0] - Mfuellst[1, 1:n] - Mfuel_additional, \
#         addedweightlst[1:n], Mfuellst[2, 0] - Mfuellst[2, 1:n] - Mfuel_additional, \
#         addedweightlst[1:n], Mfuellst[3, 0] - Mfuellst[3, 1:n] - Mfuel_additional, \
#         addedweightlst[1:n], Mfuellst[4, 0] - Mfuellst[4, 1:n] - Mfuel_additional)


range1 = Mfuellst[0, 0] - Mfuellst[0, 1:n] - Mfuel_additional
range2 = Mfuellst[1, 0] - Mfuellst[1, 1:n] - Mfuel_additional
range3 = Mfuellst[2, 0] - Mfuellst[2, 1:n] - Mfuel_additional
range4 = Mfuellst[3, 0] - Mfuellst[3, 1:n] - Mfuel_additional
range5 = Mfuellst[4, 0] - Mfuellst[4, 1:n] - Mfuel_additional

# print(range1[0], range2[0], range3[0], range4[0], range5[0])

def x_intercept(y_intercept, slope):
    return -1*y_intercept / slope

y_intercept = range1[0]

i = 1

slope1 = (range1[i] - range1[i-1])/(addedweightlst[2]-addedweightlst[1])
slope2 = (range2[i] - range2[i-1])/(addedweightlst[2]-addedweightlst[1])
slope3 = (range3[i] - range3[i-1])/(addedweightlst[2]-addedweightlst[1])
slope4 = (range4[i] - range4[i-1])/(addedweightlst[2]-addedweightlst[1])
slope5 = (range5[i] - range5[i-1])/(addedweightlst[2]-addedweightlst[1])

slopes = [slope1, slope2, slope3, slope4, slope5]
slopesr = [round(slope, 3) for slope in slopes]

x_ints = [x_intercept(y_intercept, slope) for slope in slopes]


if __name__ == '__main__':
    print(f'slope 1: {slope1}, \nslope 2: {slope2}, \nslope 3: {slope3}, \nslope 4: {slope4}, \nslope 5: {slope5}')
    print(slopesr)
    print(x_ints)
    print(y_intercept)