import numpy
avgs = {}
sums = {}
variances = {}

# see https://stackoverflow.com/questions/1969240/mapping-a-range-of-values-to-another
def mapRange(value, leftMin, leftMax, rightMin, rightMax):
    """ Translates one range into another range.
    e.g. mapRange(1, 0, 10, 0, 100) == 10
    
    :param value: the value to translate
    :param leftMin: minimum of original range of given value
    :param leftMax: maximum of original range of given value
    :param rightMin: minimum value of target range
    :param rightMax: maximum value of target range
    :return: value translated to the target range
    """
    # Figure out how 'wide' each range is
    leftSpan = leftMax - leftMin
    rightSpan = rightMax - rightMin

    # Convert the left range into a 0-1 range (float)
    valueScaled = float(value - leftMin) / float(leftSpan)

    # Convert the 0-1 range into a value in the right range.
    return rightMin + (valueScaled * rightSpan)


def movingAvg(new_sample, key, N = 10):
    """ returns a moving average of the given samples
    
    :param new_sample: a new sample to add to the moving average
    :param key: any unique string key which is used to store samples internally
    :param N: window size, default = 10
    :return: the average of the last N values passed to this function.
    """
    global avgs
    avgs = prepareObject(avgs, new_sample, key, N)
    return numpy.mean(avgs[key])
    

def movingSum(new_sample, key, N = 10):
    """ returns a moving sum of the given samples
    
    :param new_sample: a new sample to add to the moving sum
    :param key: any unique string key which is used to store samples internally
    :param N: window size, default = 10
    :return: the sum of the last N values passed to this function.
    """
    global sums
    sums = prepareObject(sums, new_sample, key, N)
    return sum(sums[key])

def movingVar(new_sample, key, N = 10):
    """ returns a moving variance of the given samples
    
    :param new_sample: a new sample to add to the moving variance
    :param key: any unique string key which is used to store samples internally
    :param N: window size, default = 10
    :return: the variance of the last N values passed to this function.
    """
    global variances
    variances = prepareObject(variances, new_sample, key, N)
    return numpy.var(variances[key])
    
def prepareObject(obj, new_sample, key, N):
    """ prepares an object for the movingXY functions of this file
    
    :param obj: dictionary object to prepare
    :param new_sample: the new sample to add to obj[key]
    :param key: any unique string key which is used to store samples internally
    :param N: window size
    :return: the given object with object[key] with new_sample added to
             the array and the array limited to N elements
    """
    if not key in obj:
        obj[key] = []
    while len(obj[key]) >= N:
        obj[key].pop(0)
    obj[key].append(new_sample)
    return obj
    
def limit(value, minimum, maximum):
    """ limits a value to a given min/max
    
    :param value: the value to limit
    :param minimum: min value to return
    :param maximum: max value to return
    :return: value or min/max if value is smaller/bigger than min/max
    """
    return max(min(value, maximum), minimum)
    
    
def rotationDegreeToTime(degrees, speed):
    """ calculates how much time the robot has to rotate at a given
    speed to rotate around a certain angle
    
    :param degrees: the angle in degrees to rotate
    :param speed: the rotation speed
    :return: the time to rotate
    """
    constFactorFullRotation = 6.74 # measured, see doc/measurements.xlsx
    timeFullRotation = constFactorFullRotation / speed
    return timeFullRotation / 360 * degrees
