'''
    Karthik Praturu
    Wifi Localization
'''

import objc
import time
import matplotlib.pyplot as plt
import numpy as np
import heapq


# Copied from stackoverflow solution to determining wifi data on OS X
# source: https://stackoverflow.com/questions/15169022/is-there-any-way-to-access-os-x-wi-fi-data-using-python-signal-strength-for-e/15170342
objc.loadBundle('CoreWLAN',
                bundle_path='/System/Library/Frameworks/CoreWLAN.framework',
                module_globals=globals())
def findWifi():
    for iname in CWInterface.interfaceNames():
      interface = CWInterface.interfaceWithName_(iname)
      print """
    Interface:      %s
    SSID:           %s
    Transmit Rate:  %s
    Transmit Power: %s
    RSSI:           %s""" % (iname, interface.ssid(), interface.transmitRate(),
                             interface.transmitPower(), interface.rssi())
    return interface


def getDistance(interface):
    # Assumes measured dbm of -80 at 50 m away from AP
    # Assumes n = 3
    rssi = interface.rssi()
    beta_numerator = float(-80-rssi)
    beta_denominator = float(10*3)
    beta = beta_numerator/beta_denominator
    distanceFromAP = round(((10**beta)*50),4)
    print "Distance: " + str(distanceFromAP)
    return (distanceFromAP, rssi, 0)

num_samps = 0
curAvg = 0
class DistanceEstimator:
    def __init__(self):
        self.curAvg = 0
        self.num_samps = 0

    def getDistance(self, interface, toPrint):
        rssi = interface.rssi()
        beta_numerator = float(-80-rssi)
        beta_denominator = float(10*3)
        beta = beta_numerator/beta_denominator
        distanceFromAP = round(((10**beta)*50),4)

        if toPrint:
            print "Distance: " + str(distanceFromAP)
        return (distanceFromAP, rssi, 0)

    def getLSEDistance(self, interface, toPrint):
        self.num_samps += 1
        rssi = interface.rssi()
        self.curAvg = float(rssi+80)/self.num_samps + self.curAvg*(self.num_samps-1)/self.num_samps
        A = -1.0/(10*3)*(self.curAvg)
        distanceFromAP = round(((10**A)*50), 4)

        if toPrint:
            print "Estimated Distance: " + str(distanceFromAP)
            print "RSSI: " + str(rssi)
        return (distanceFromAP, rssi, self.curAvg)


    def getDistanceSim(self, dev_loc, toPrint):
        # sim_loc at (-8, -4)
        sim_x = -6
        sim_y = -4
        (dev_x, dev_y) = dev_loc
        dist = np.sqrt(np.abs(sim_x - dev_x)**2 + np.abs(sim_y - dev_y)**2)
        sigma = dist*np.sqrt(2/np.pi)

        distanceFromAP = np.random.rayleigh(sigma)

        if toPrint:
            print "Randomized Distance: " + str(distanceFromAP)
            print "Actual Distance: " + str(dist)
        return (distanceFromAP, dist, 0)

    def resetSamps(self):
        self.curAvg = 0
        self.num_samps = 0


class Localizer:
    def __init__(self, numIter, sampSize):
        self.numIter = numIter
        self.x = 0
        self.y = 0
        self.samp_size = sampSize

        grid_x = np.arange(-25, 25, 0.5)
        grid_y = np.arange(-25, 25, 0.5)
        self.grid = []

        for x in grid_x:
            for y in grid_y:
                # uniform prior
                self.grid.append((x, y, 1))

    def localize(self):
        ap_est = DistanceEstimator()
        iface = findWifi()
        iterations = 0
        while iterations < self.numIter:
            input = raw_input("Input movement direction (wasd): ")

            prev_x = self.x
            prev_y = self.y

            if input == 'w':
                self.y += 1
            elif input == 'a':
                self.x -= 1
            elif input == 's':
                self.y -= 1
            elif input == 'd':
                self.x += 1

            if not (prev_x == self.x and prev_y == self.y):
                ap_est.resetSamps()

            distanceFromAP, rssi, _ = ap_est.getLSEDistance(iface, False)

            # Now do Bayesian analysis to determine MLE points
            mle_locs = self.determineMLE(distanceFromAP, 10)

            mle_x = []
            mle_y = []
            for x, y, prob in mle_locs:
                mle_x.append(x)
                mle_y.append(y)

            print "My loc: " + str((self.x, self.y))
            print "MLE AP loc: " + str(mle_locs[0])
            plt.clf()
            plt.scatter(mle_x, mle_y, color = 'blue')
            plt.scatter(self.x, self.y, color = 'red')
            plt.xlim(-10, 10)
            plt.ylim(-10, 10)
            plt.xlabel("meters")
            plt.ylabel("meters")
            plt.savefig(str(iterations) + "_iter.png")
            plt.show()
            iterations += 1

    def sim_localize(self):
        ap_est = DistanceEstimator()
        iterations = 0
        while True:
            input = raw_input("[" + str(iterations) + "] Input movement direction (wasd): ")

            prev_x = self.x
            prev_y = self.y

            if input == 'w':
                self.y += 1
            elif input == 'a':
                self.x -= 1
            elif input == 's':
                self.y -= 1
            elif input == 'd':
                self.x += 1
            elif input == 'clear':
                self.x = 0
                self.y = 0

                grid_x = np.arange(-25, 25, 0.5)
                grid_y = np.arange(-25, 25, 0.5)

                self.grid = []

                for x in grid_x:
                    for y in grid_y:
                        # uniform prior
                        self.grid.append((x, y, 1))
                continue

            elif input == 'q':
                break

            if not (prev_x == self.x and prev_y == self.y):
                ap_est.resetSamps()

            # Simulated AP at location -6, -4
            distanceFromAP, _, _ = ap_est.getDistanceSim((self.x, self.y), True)

            # Now do Bayesian analysis to determine MLE points
            mle_locs = self.determineMLE(distanceFromAP, 10)

            mle_x = []
            mle_y = []
            for x, y, prob in mle_locs:
                mle_x.append(x)
                mle_y.append(y)

            print "My loc: " + str((self.x, self.y))
            print "MLE AP loc: " + str(mle_locs[0])
            plt.clf()
            plt.scatter(mle_x, mle_y, color = 'blue')
            plt.scatter(self.x, self.y, color = 'red')
            plt.xlim(-10, 10)
            plt.ylim(-10, 10)
            plt.xlabel("meters")
            plt.ylabel("meters")
            plt.savefig("simiter.png")
            plt.show(block=False)
            iterations += 1
    # Assuming Rayleigh Distribution with std. dev of 1
    def determineMLE(self, distance, n):
        sigma = 1

        prob_nums = []
        for i in range(len(self.grid)):
            (x, y, prior) = self.grid[i]

            sample_dist = np.sqrt(np.abs(self.x - x)**2 + np.abs(self.y - y)**2)
            dist_diff = np.abs(sample_dist - distance)
            prob_nums.append(dist_diff*np.exp(-np.power(dist_diff, 2))*prior)

        norm_factor = sum(prob_nums)
        dist = [x/norm_factor for x in prob_nums]
        self.grid = [(self.grid[i][0], self.grid[i][1], dist[i]) for i in range(len(self.grid))]

        return heapq.nlargest(n, self.grid, key = lambda x: x[2])

def getEstimatedDistance(interface, curAvg):
    #Assumes measured dbm of -80 at 50 m away from AP
    # Assumes n = 3
    global num_samps, n
    num_samps+=1
    rssi = interface.rssi()
    curAvg = float(rssi+80)/num_samps + curAvg*(num_samps-1)/num_samps
    A = -1.0/(10*3)*(curAvg)
    distanceFromAP = round(((10**A)*50), 4)
    print "Estimated Distance: " + str(distanceFromAP)
    print "RSSI: " + str(rssi)
    return (distanceFromAP, rssi, curAvg)


'''
iface = findWifi()
regDists = []
estimatedDists = []
rssis = []
ap_est = DistanceEstimator()
num_samps = 0
while(num_samps < 10):
    time.sleep(3)
    (dist, rssi, _) = ap_est.getDistance(iface, True)
    regDists.append(dist)
    rssis.append(rssi)
    (estDist,erssi, curAvg) = ap_est.getLSEDistance(iface, True)
    estimatedDists.append(estDist)

    num_samps += 1


plt.plot(rssis, estimatedDists, '.')
plt.title("RSS vs Measured Distance")
plt.xlabel("RSS (dBm)")
plt.ylabel("Distance (m)")
plt.show()

plt.figure()
plt.plot(range(10), regDists, estimatedDists, ".-")
plt.title("Measured Distance vs. Time")
plt.xlabel("Sample Number")
plt.ylabel("Measured Distance (m)")
plt.legend(['Direct Calc', 'LSE'])
plt.show()
'''

localizer = Localizer(10, 100)

localizer.sim_localize()
