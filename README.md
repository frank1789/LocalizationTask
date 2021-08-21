# Task 

This particular task requires found a possible algorithm to localize a car using the data contained in the files.

## preliminary operation
First, it was necessary to parse every single file to retrieve data and properly manipulate them.
Due to the presence of the timestamp collected by system as Linux time stamp, I converted it in the difference of time using a standard library of python and NumPy 

```python
#original data timestamp every [ns] to [s]
    t0=self.groundtruth_data[0, 0] // 1e9
    for i in range(1, len(self.groundtruth_data[:, 0])):
        t = datetime.utcfromtimestamp(self.groundtruth_data[i, 0] // 1e9) - datetime.utcfromtimestamp(t0)
        self.groundtruth_data[i, 0] = t.total_seconds()
```
The code is shown before performing the action to retrieve information about the time elapsed in s.

## The idea behind the estimator

To perform the localization of the car, I have decided to use the **Extended Kalman Filter**, because it is well known in the literature and it is an optimal estimator.
The model of the car is not furnished, so it is impossible to perform a kinematic calibration 
to retrieve the wheels radius and track axle, which could be useful to make a better kinematic model used in EKF.

So, I introduced some noise on the parameters as request used the value furnished, in other cases I used other suitable values.

## The GNSS data

The GNSS data information was used as a measurement of the car's position.
I believe could be necessary a filter before using it in the estimator.
These measurements are used as correction of predictions in EKF, before described


## The ground truth

The data collected in this file was used as the initial car's position. 
In my idea, it would possibly extract the angles measurements expressed as quaternions and converted them into homogeneous matrices to achieve the angles of transformation between the car's Reference Frame and Word Reference frame.
I'm not to able perform this kind of operation because I've never used before the quaternion property to exchange from a representation to an other.

## Overall result

The localization algorithm does not perform as expected due to the presence of interval time repeated. 
The car's model probably is not correct and this makes an invalid prediction in my estimator. Using a complete and better model should be producing a more reliable prediction.
