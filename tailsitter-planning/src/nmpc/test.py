def getNormalizedThrust(thrust: float):
    if (thrust > 16.):
        print("WARN:MAX_THRUST")
        return 1.
    if (thrust < 0.):
        print("WARN:MIN_THRUST")
        return 0.
    a = 5.40732857
    b = 11.02346575
    return ((b*b + 4 * a * thrust) ** (1/2) - b)/(2*a)


print(getNormalizedThrust(9.8))