## Code used in the Great Drone Duel of 2016

### Authors

Logan Lamb, Ben Morgan, Marc Newlin

### Background

At ToorCamp 2016, an unknown Chinese benefactor provided all participants with Cheerson CX-10A quadcopters. Coincidentally,
Michael Ossmann and Dominic Spill gave a talk about hacking those very same quadcopters, and as part of their talk, they released 
a protocol specification which formalized the packet format used by the drones. 

Following the only logical path that made sense at the time, we [challenged them to a duel](https://twitter.com/marcnewlin/status/741401358465519616) at high noon. 

Using Python, nRF24LU1+ dongles (running [Marc's nRF24LU1+ firmware](https://github.com/RFStorm/mousejack)), 
and an IntimidationAntenna(tm), we hacked together some code to either fly their drones far, far away, or bring them crashing 
to the ground.

The code has been alpha tested against giant fishing nets with mixed results. 

### Hardware

- Cheerson CX-10A 
- CrazyRadio PA dongle (or other nRF24LU1+ based USB dongle or breakout board)
- IndimidationAntenna(tm), or stock antenna 
- Giant fishing net (optional) 

### Usage 

#### 1. Clone the repository and submodule 
```
git clone https://github.com/marcnewlin/drone-duel.git
cd drone-duel
git submodule init
git submodule update
```

#### 2. Flash your CrazyRadio PA dongle 
```
cd mousejack
make
sudo make install
cd ..
```

#### 3. Fly, fly away! (Pair with a drone and cause it to fly away)
```
sudo ./fly-fly-away.py 
```

#### 4. Rain from the sky! (Hijack a drone in-flight and cause it to come crashing to the ground) 
```
sudo ./rain-from-the-sky.py
```
