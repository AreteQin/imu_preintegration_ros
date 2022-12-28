from Quanser.product_QCar import QCar
import time
import struct
import numpy as np 

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
## Timing Parameters and methods 
startTime = time.time()
def elapsed_time():
    return time.time() - startTime

sampleRate = 1000
sampleTime = 1/sampleRate
simulationTime = 5.0
print('Sample Time: ', sampleTime)

IMU_Data  = np.zeros(6, dtype=np.float64)

# Additional parameters
counter = 0

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
## QCar Initialization
myCar = QCar()

# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 
## Main Loop
try:
    while elapsed_time() < simulationTime:
        # Start timing this iteration
        start = time.time()
   

        myCar.read_IMU()
        gyro_data = myCar.read_other_buffer_IMU[0:3]
        accel_data = myCar.read_other_buffer_IMU[3:6]
        print("Gyro data in x:{}, y:{}, z:{}".format(gyro_data[0],gyro_data[1],gyro_data[2]))
        print("Accel data in x:{}, y:{}, z:{}".format(accel_data[0],accel_data[1],accel_data[2]))
        # End timing this iteration
        end = time.time()

        # Calculate computation time, and the time that the thread should pause/sleep for
        computation_time = end - start
        sleep_time = sampleTime - computation_time%sampleTime

        # Pause/sleep and print out the current timestamp
        time.sleep(sleep_time)
        counter += 1

except KeyboardInterrupt:
    print("User interrupted!")

finally:    
    myCar.terminate()
# -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- 