# ----------
# Particles Filter

from robot import *
from math import *
from matrix import *
import random

# calculates the probability of x for 1-dim Gaussian with mean mu and var. sigma
def Gaussian(mu, sigma, x):
    return exp(- ((mu - x) ** 2) / (sigma ** 2) / 2.0) / sqrt(2.0 * pi * (sigma ** 2))


def measurement_prob(p, measurement):
    return Gaussian(distance_between((p.x, p.y), (0,0)), p.measurement_noise,
                    distance_between(measurement, (0,0)))

def distance_between(point1, point2):
    """Computes distance between point1 and point2. Points are (x, y) pairs."""
    x1, y1 = point1
    x2, y2 = point2
    return sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# Estimate next position of the tracking robot using particles filter.
def estimate_next_pos(measurement, OTHER = None):
    """Estimate the next (x, y) position of the wandering Traxbot
    based on noisy (x, y) measurements."""

    xy_estimate = measurement

    if OTHER is None:
        OTHER = {'step': 1,
                 'measurements': [measurement]
                 }
    elif OTHER['step'] < 5:  # record several measurements first
        OTHER['measurements'].append(measurement)
        OTHER['step'] += 1

    elif OTHER['step'] == 5:

        # guess the movement of the robot by averaging the moving
        # turning and distance for the measurements recorded in
        # previous steps
        measurements = OTHER['measurements']
        distance = 0
        turning = 0
        prev_heading = None
        for i in range(1, len(measurements)):
            distance += distance_between(measurements[i], measurements[i - 1])

            heading = atan2(measurements[i][1] - measurements[i - 1][1],
                            measurements[i][0] - measurements[i - 1][0])
            if prev_heading is not None:
                turning += (heading - prev_heading)

            prev_heading = heading

        distance /= len(measurements) - 1
        turning /= len(measurements) - 2

        # Make particles
        p = []
        for _ in range(500):
            # make particle robots with random turning and distance
            r = robot(measurement[0], measurement[1], heading,
                      random.gauss(turning, 0.1 * turning),
                      random.gauss(distance, 0.01 * distance))
            r.set_noise(0, 0, 5) # set noise for measurement prob
            p.append(r)
        OTHER['particles'] = p

        # Save the measurement for the next prediction
        OTHER['step'] += 1
        OTHER['prev_measurement'] = measurement

    else:

        #
        # Apply particle filter
        #

        # motion update
        for r in OTHER['particles']:
            r.wander()

        # measurement update
        w = [measurement_prob(p, measurement) for p in OTHER['particles']]

        # resampling
        N = len(OTHER['particles'])
        p3 = []
        index = int(random.random() * N)
        beta = 0.0
        mw = max(w)
        for i in range(N):
            beta += random.random() * 2.0 * mw
            while beta > w[index]:
                beta -= w[index]
                index = (index + 1) % N
            p3.append(OTHER['particles'][index])
        OTHER['particles'] = p3

        # set particle position and heading according to the measurement
        # for prediction
        heading = atan2(measurement[1] - OTHER['prev_measurement'][1],
                        measurement[0] - OTHER['prev_measurement'][0])
        for r in OTHER['particles']:
            r.x = measurement[0]
            r.y = measurement[1]
            r.heading = heading

        # Save the measurement for the next prediction
        OTHER['prev_measurement'] = measurement

        # prediction by having all the particles moving one step forward
        # then average there positions to get the estimated position
        x = 0
        y = 0
        for r in OTHER['particles']:
            # make a copy of each particle so that move function does not
            # effect the particle itself
            r2 = robot(r.x, r.y, r.heading, r.turning, r.distance)
            r2.wander()
            x += r2.x
            y += r2.y
        xy_estimate = [x / N, y /N]

    return xy_estimate, OTHER


def train(measurements):
    """Train the particles filter using the given measurements"""

    OTHER = None
    for measurement in measurements:
        position_guess, OTHER = estimate_next_pos(measurement, OTHER)
        # TODO - add visualization to compare estimated positions and measurements

    # return filtered/trained particles
    return OTHER['particles']


def predict(particles, num_frames):
    """Predict next *num_frames* positions using the given particles"""

    num_particles = len(particles)

    extra_frames = []
    for _ in range(num_frames):

        # motion update for all particles
        for p in particles:
            p.wander()

        # get prediction by averaging particles' positions
        x = sum(p.x for p in particles) / num_particles
        y = sum(p.y for p in particles) / num_particles

        extra_frames.append([x,y])

    return extra_frames
