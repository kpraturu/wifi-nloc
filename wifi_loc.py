import objc
import time
import matplotlib.pyplot as plt

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

iface = findWifi()

regDists = []
estimatedDists = []
rssis = []
while(num_samps < 2):
    time.sleep(3)
    (dist, rssi, _) = getDistance(iface)
    regDists.append(dist)
    rssis.append(rssi)
    (estDist,erssi, curAvg) = getEstimatedDistance(iface, curAvg)
    estimatedDists.append(estDist)

plt.plot(rssis, estimatedDists, '.')
plt.title("RSS vs Measured Distance")
plt.xlabel("RSS (dBm)")
plt.ylabel("Distance (m)")
plt.show()
