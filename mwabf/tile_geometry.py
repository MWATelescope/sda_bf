
import math


def calc_delays(az=0.0, el=0.0):
    """
       Function calc_delays
       This function takes in an azimuth and zenith angle as
       inputs and creates and returns a 16-element byte array for
       delayswitches which have values corresponding to each
       dipole in the tile having a maximal coherent amplitude in the
       desired direction.

       This will return null if the inputs are
       out of physical range (if za is bigger than 90) or
       if the calculated switches for the dipoles are out
       of range of the delaylines in the beamformer.

       azimuth of 0 is north and it increases clockwise
       zenith angle is the angle down from zenith
       These angles should be given in degrees

      Layout of the dipoles on the tile:

                 N

           0   1   2   3

           4   5   6   7
      W                    E
           8   9   10  11

           12  13  14  15

                 S
    """
    dip_sep = 1.10  # dipole separations in meters
    delaystep = 435.0  # Delay line increment in picoseconds
    maxdelay = 31  # Maximum number of deltastep delays
    c = 0.000299798  # C in meters/picosecond
    dtor = math.pi / 180.0  # convert degrees to radians
    # define zenith angle
    za = 90 - el

    # Define arrays to hold the positional offsets of the dipoles
    xoffsets = [0.0] * 16  # offsets of the dipoles in the W-E 'x' direction
    yoffsets = [0.0] * 16  # offsets of the dipoles in the S-N 'y' direction
    delays = [0.0] * 16  # The calculated delays in picoseconds
    rdelays = [0] * 16  # The rounded delays in units of delaystep

    delaysettings = [0] * 16  # return values

    # Check input sanity
    if (abs(za) > 90):
        return delaysettings

        # Offsets of the dipoles are calculated relative to the
        # center of the tile, with positive values being in the north
        # and east directions

    xoffsets[0] = -1.5 * dip_sep
    xoffsets[1] = -0.5 * dip_sep
    xoffsets[2] = 0.5 * dip_sep
    xoffsets[3] = 1.5 * dip_sep
    xoffsets[4] = -1.5 * dip_sep
    xoffsets[5] = -0.5 * dip_sep
    xoffsets[6] = 0.5 * dip_sep
    xoffsets[7] = 1.5 * dip_sep
    xoffsets[8] = -1.5 * dip_sep
    xoffsets[9] = -0.5 * dip_sep
    xoffsets[10] = 0.5 * dip_sep
    xoffsets[11] = 1.5 * dip_sep
    xoffsets[12] = -1.5 * dip_sep
    xoffsets[13] = -0.5 * dip_sep
    xoffsets[14] = 0.5 * dip_sep
    xoffsets[15] = 1.5 * dip_sep

    yoffsets[0] = 1.5 * dip_sep
    yoffsets[1] = 1.5 * dip_sep
    yoffsets[2] = 1.5 * dip_sep
    yoffsets[3] = 1.5 * dip_sep
    yoffsets[4] = 0.5 * dip_sep
    yoffsets[5] = 0.5 * dip_sep
    yoffsets[6] = 0.5 * dip_sep
    yoffsets[7] = 0.5 * dip_sep
    yoffsets[8] = -0.5 * dip_sep
    yoffsets[9] = -0.5 * dip_sep
    yoffsets[10] = -0.5 * dip_sep
    yoffsets[11] = -0.5 * dip_sep
    yoffsets[12] = -1.5 * dip_sep
    yoffsets[13] = -1.5 * dip_sep
    yoffsets[14] = -1.5 * dip_sep
    yoffsets[15] = -1.5 * dip_sep

    # First, figure out the theoretical delays to the dipoles
    # relative to the center of the tile

    # Convert to radians
    azr = az * dtor
    zar = za * dtor

    for i in range(16):
        # calculate exact delays in picoseconds from geometry...
        delays[i] = (xoffsets[i] * math.sin(azr) + yoffsets[i] * math.cos(azr)) * math.sin(zar) / c

    # Find minimum delay
    mindelay = min(delays)

    # Subtract minimum delay so that all delays are positive
    for i in range(16):
        delays[i] -= mindelay

    # Now minimize the sum of the deviations^2 from optimal
    # due to errors introduced when rounding the delays.
    # This is done by stepping through a series of offsets to
    # see how the sum of square deviations changes
    # and then selecting the delays corresponding to the min sq dev.

    # Go through once to get baseline values to compare
    bestoffset = -0.45 * delaystep
    minsqdev = 0

    for i in range(16):
        delay_off = delays[i] + bestoffset
        intdel = int(round(delay_off / delaystep))

        if (intdel > maxdelay):
            intdel = maxdelay

        minsqdev += math.pow((intdel * delaystep - delay_off), 2)

    minsqdev = minsqdev / 16

    offset = (-0.45 * delaystep) + (delaystep / 20.0)
    while offset <= (0.45 * delaystep):
        sqdev = 0
        for i in range(16):
            delay_off = delays[i] + offset
            intdel = int(round(delay_off / delaystep))

            if (intdel > maxdelay):
                intdel = maxdelay
            sqdev = sqdev + math.pow((intdel * delaystep - delay_off), 2)

        sqdev = sqdev / 16
        if (sqdev < minsqdev):
            minsqdev = sqdev
            bestoffset = offset

        offset += delaystep / 20.0

    for i in range(16):
        rdelays[i] = int(round((delays[i] + bestoffset) / delaystep))
        if (rdelays[i] > maxdelay):
            if (rdelays[i] > maxdelay + 1):
                return None  # Trying to steer out of range.
            rdelays[i] = maxdelay

    # Set the actual delays
    for i in range(16):
        delaysettings[i] = int(rdelays[i])

    return delaysettings


def triangulate(d1, ox1, oy1, d2, ox2, oy2, d3, ox3, oy3):
    """
    Chris Williams' function copied from receiverStatusPy/StatusTable.py

    Takes three dipole settings and triangulates to return a single (az,za) estimate.
    :param d1:
    :param ox1:
    :param oy1:
    :param d2:
    :param ox2:
    :param oy2:
    :param d3:
    :param ox3:
    :param oy3:
    :return:  A single (az,za) tuple of azimuth and zenith angle
    """
    dtor = 0.0174532925
    c = 0.000299798  # c in m/picosecond
    az = math.atan2((d3 - d1) * (oy2 - oy1) - (d2 - d1) * (oy3 - oy1),
                    (d2 - d1) * (ox3 - ox1) - (d3 - d1) * (ox2 - ox1))

    if (d1 == d2) and (d1 == d3):
        return 0.0, 0.0

    if abs((ox2 - ox3) * math.sin(az) + (oy2 - oy3) * math.cos(az)) > 1e-15:  # check if the triangle is bad (if its colinear)
        za = math.asin((d2 - d3) * c / ((ox2 - ox3) * math.sin(az) + (oy2 - oy3) * math.cos(az)))
    elif abs((ox1 - ox3) * math.sin(az) + (oy1 - oy3) * math.cos(az)) > 1e-15:
        za = math.asin((d1 - d3) * c / ((ox1 - ox3) * math.sin(az) + (oy1 - oy3) * math.cos(az)))
    else:
        # print("Bad delays in tilestatus.triangulate(): %d, %d, %d", (d1, d2, d3))
        return None, None
    azd = az / dtor
    zad = za / dtor

    if zad < 0:
        zad *= -1
        azd += 180
    while azd < 0:
        azd += 360
    while azd >= 360:
        azd -= 360
    return azd, zad


def delays2azel(xx):
    """
    Chris Williams' function copied from receiverStatusPy/StatusTable.py

    Takes a list of dipole delays and returns a tuple of (azimuth,elevation).

    If the values can't be computed from the delays, return az=0, el=90

    :param xx: a list of 16 dipole delays
    :return: a tuple of (azimuth,elevation)
    """
    dip_sep = 1.10
    delaystep = 435  # delay in picoseconds
    # dtor = 0.0174532925

    for d in xx:
        if d is None:
            return 0.0, 90.0

    # choose triangles to back out the delays...
    azs = []
    zas = []

    ii = [0, 0, 3, 0]
    jj = [15, 15, 12, 3]
    kk = [12, 3, 15, 12]

    for a in range(len(ii)):
        i = ii[a]
        j = jj[a]
        k = kk[a]

        d1 = delaystep * xx[i]
        ox1 = (-1.5 + (i % 4) * 1.0) * dip_sep
        oy1 = (1.5 - math.floor(i / 4)) * dip_sep

        d2 = delaystep * xx[j]
        ox2 = (-1.5 + (j % 4) * 1.0) * dip_sep
        oy2 = (1.5 - math.floor(j / 4)) * dip_sep

        d3 = delaystep * xx[k]
        ox3 = (-1.5 + (k % 4) * 1.0) * dip_sep
        oy3 = (1.5 - math.floor(k / 4)) * dip_sep

        try:
            az, za = triangulate(d1, ox1, oy1, d2, ox2, oy2, d3, ox3, oy3)
        except:  # AAVS delays can give math errors
            az, za = 0.0, 0.0

        if az is not None:
            azs.append(az)
            zas.append(za)
        else:  # Bad triangle...
            pass
            # print("Bad delay triangle: (%d, %d, %d)", (i, j, k))

    if len(azs):
        azavg = sum(azs) / len(azs)
    else:
        azavg = 0

    if len(zas):
        zaavg = sum(zas) / len(zas)
    else:
        zaavg = 90.0

    return round(azavg, 1), round(90 - zaavg, 1)
