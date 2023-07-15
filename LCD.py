#!/usr/bin/python3
import sys
sys.path.append('/home/roy951/.local/lib/python3.9/site-packages/cv2')

from gps import *
import time
from time import sleep
import board
from digitalio import DigitalInOut
from adafruit_character_lcd.character_lcd import Character_LCD_Mono
import Adafruit_DHT
from datetime import datetime
import smbus
import math
import threading

# Configuration du capteur + Numéro de la broche GPIO à laquelle le capteur est connecté
capteur = Adafruit_DHT.DHT22
pin =17 

# Modify this if you have a different sized character LCD
lcd_columns = 16
lcd_rows = 2

lcd_rs = DigitalInOut(board.D26)
lcd_en = DigitalInOut(board.D19)
lcd_d4 = DigitalInOut(board.D13)
lcd_d5 = DigitalInOut(board.D6)
lcd_d6 = DigitalInOut(board.D5)
lcd_d7 = DigitalInOut(board.D11)

# Initialise the LCD class
lcd = Character_LCD_Mono(
    lcd_rs, lcd_en, lcd_d4, lcd_d5, lcd_d6, lcd_d7, lcd_columns, lcd_rows
)

#Verrou thread
lcd_lock = threading.Lock()

# power management register
power_mgmt_1 = 0x6b
power_mgmt_2 = 0x6c

base_x_accel= 0 
base_y_accel= 0 
base_z_accel= 0 
base_x_gyro = 0 
base_y_gyro = 0 
base_z_gyro = 0 

last_read_time = 0
last_x_angle = 0
last_y_angle = 0
last_z_angle = 0
last_gyro_x_angle = 0
last_gyro_y_angle = 0
last_gyro_z_angle = 0

bus = 0
started = 0
address = 0

start_time = time.time()
timeout = 60 # Temps en secondes après lequel la boucle se termine


def exit_program():
    global running
    running = False

def handle_exit():
    time.sleep(3)
    print('Exiting GPS...')
    #fichier.close()
    sys.exit(0)
    
# Register signal handler for Ctrl+C
#signal.signal(signal.SIGINT, lambda signal, frame: exit_program())

def CM():
    # Lecture des données du capteur
    humidite, temperature = Adafruit_DHT.read_retry(capteur, pin)
    heure_utc = datetime.utcnow()
    if temperature is not None and humidite is not None:
        sleep(2)
        print("---------------------------------------------------------------------")
        print('Température = {0:0.1f}°C  Humidité = {1:0.1f}%'.format(temperature, humidite))
        print("Heure UTC :", heure_utc)
    
    
    with lcd_lock:
        if temperature is not None and humidite is not None:
            lcd.message = "Temp={0:0.1f}C\n Humidite={1:0.1f}%".format(temperature, humidite)
            sleep(2)
            lcd.clear() 
    

def getPositionData(gps):
    nx = gpsd.next()
    if nx['class'] == 'TPV':
        latitude = getattr(nx,'lat',"Signal cut")
        longitude = getattr(nx,'lon',"Signal cut")
        time = getattr(nx,'time',"Unknown")
        
        print("---------------------------------------------------------------------")
        print("Your position : longitude = " + str(longitude) + "°E, latitude = " +str(latitude) + "°N, time = " +str(time))
        
        with lcd_lock:
            lcd.message = f"Long:{longitude}°E\n Lat:{latitude}°N"
            sleep(.5)
            for x in range(0, 16):
                lcd.move_left()
                sleep(.2)
            sleep(2)
    
            # scroll text on display
            for x in range(0, 16):
                lcd.move_right()
                sleep(.2)
            sleep(2)
            lcd.clear()
        

def read_byte(adr):
    return bus.read_byte_data(address, adr)

def read_word(adr):
    high = bus.read_byte_data(address, adr)
    low = bus.read_byte_data(address, adr+1)
    val = (high << 8) + low
    return val

#cause it max 16 bit data bit python is further more than that
def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return - (0x10000 - val)
    else :
        return val

def dist(a,b):
    result = math.sqrt((a*a)+(b*b))
    if result != 0:
        return result
    return 0.001

def get_y_rotation(x,y,z):
    radians = math.atan2(x, dist(y,z))
    return -math.degrees(radians)

def get_x_rotation(x,y,z):
    radians = math.atan2(y, dist(x,z))
    return math.degrees(radians)

def set_last_read_angle_data(time, x, y, z, x_gyro, y_gyro, z_gyro):
    global last_read_time
    global last_x_angle 
    global last_y_angle 
    global last_z_angle 
    global last_gyro_x_angle
    global last_gyro_y_angle
    global last_gyro_z_angle

    last_read_time = time
    last_x_angle = x
    last_y_angle = y
    last_z_angle = z
    last_gyro_x_angle = x_gyro
    last_gyro_y_angle = y_gyro
    last_gyro_z_angle = z_gyro

def calibrate_sensors():
    if started == 0:
        return {"result":-5}
    global base_x_accel 
    global base_y_accel 
    global base_z_accel 
    global base_x_gyro  
    global base_y_gyro  
    global base_z_gyro  

    num_read = 20
    x_accel = 0
    y_accel = 0
    z_accel = 0
    x_gyro = 0
    y_gyro = 0
    z_gyro = 0

    print("Calibrating...")
    for i in range(0,num_read):
        x_gyro += read_word_2c(0x43)
        y_gyro += read_word_2c(0x45)
        z_gyro += read_word_2c(0x47)
        x_accel += read_word_2c(0x3b)
        y_accel += read_word_2c(0x3d)
        z_accel += read_word_2c(0x3f)
        time.sleep(0.1)

    base_x_accel = x_accel/num_read
    base_y_accel = y_accel/num_read 
    base_z_accel = z_accel/num_read 
    base_x_gyro  = x_gyro/num_read 
    base_y_gyro  = y_gyro/num_read  
    base_z_gyro  = z_gyro/num_read  
    print("Done.")

def initial_sensor_module():
    global started
    global bus
    global address
    bus = smbus.SMBus(1)
    address = 0x68 #MPU6050 I2C adress
    started = 1
    
    #MPU6050 is in sleep mode so awake in first timeuse:
    bus.write_byte_data(address, power_mgmt_1, 0)
    
    calibrate_sensors()
    set_last_read_angle_data(datetime.now(), 0, 0, 0, 0, 0, 0)

def read_filtered_out():
    if started == 0:
        return {"result":-5}
    try:
        temp_out = read_word_2c(0x41)
    
        gyro_xout = read_word_2c(0x43)
        gyro_yout = read_word_2c(0x45)
        gyro_zout = read_word_2c(0x47)
        
        accel_xout = read_word_2c(0x3b)
        accel_yout = read_word_2c(0x3d)
        accel_zout = read_word_2c(0x3f)

        t_now = datetime.now()

        FS_SEL = 131
        gyro_xout = (gyro_xout - base_x_gyro)/FS_SEL
        gyro_yout = (gyro_yout - base_y_gyro)/FS_SEL
        gyro_zout = (gyro_zout - base_z_gyro)/FS_SEL
        
        accel_xout_scaled = accel_xout / 16384.0
        accel_yout_scaled = accel_yout / 16384.0
        accel_zout_scaled = accel_zout / 16384.0
        
        RADIANS_TO_DEGREES = 180/math.pi

        accel_angle_y = math.atan(-1*accel_xout/ dist(accel_xout,accel_zout))*RADIANS_TO_DEGREES
        accel_angle_x = math.atan(accel_yout/ dist(accel_yout,accel_zout))*RADIANS_TO_DEGREES
        accel_angle_z = 0

        # Compute the (filtered) gyro angles
        dt = (t_now - last_read_time).microseconds/1000000
        gyro_angle_x = gyro_xout*dt + last_x_angle
        gyro_angle_y = gyro_yout*dt + last_y_angle
        gyro_angle_z = gyro_zout*dt + last_z_angle

        # Compute the drifting gyro angles
        unfiltered_gyro_angle_x = gyro_xout*dt + last_gyro_x_angle
        unfiltered_gyro_angle_y = gyro_yout*dt + last_gyro_y_angle
        unfiltered_gyro_angle_z = gyro_zout*dt + last_gyro_z_angle

        # Apply the complementary filter to figure out the change in angle
        # - choice of alpha is estimated now.
        # Alpha depends on the sampling rate...
        alpha = 0.96
        angle_x = alpha * gyro_angle_x + (1.0 - alpha)*accel_angle_x
        angle_y = alpha * gyro_angle_y + (1.0 - alpha)*accel_angle_y
        angle_z = gyro_angle_z # Accelerometer doesn't given z-angle

        # Update the saved data with the lastest values
        set_last_read_angle_data(t_now, angle_x, angle_y, angle_z,\
                gyro_angle_x, gyro_angle_y, gyro_angle_z)
        data = {
            "result": 0,
            "delta_time":dt,
            "gyro_angle_x":gyro_angle_x,
            "gyro_angle_y":gyro_angle_y,
            "gyro_angle_z":gyro_angle_z,
            "accel_angle_x":accel_angle_x,
            "accel_angle_y":accel_angle_y,
            "accel_angle_z":accel_angle_z,
            "angle_x":angle_x,
            "angle_y":angle_y,
            "angle_z":angle_z
        }
        return data
    except IOError:
        KeyboardInterrupt('An error occured trying to read..(IOError)')
    except KeyboardInterrupt:
        raise KeyboardInterrupt('The operation have been cancel by keyboard.(KeyboardInterrupt)')

def MPU():
    
    try:
        #read tempurature
        temp_out = read_word_2c(0x41)
    
        #Gyro register read
        gyro_xout = read_word_2c(0x43)
        gyro_yout = read_word_2c(0x45)
        gyro_zout = read_word_2c(0x47)
                
        #Accelaration register read
        accel_xout = read_word_2c(0x3b)
        accel_yout = read_word_2c(0x3d)
        accel_zout = read_word_2c(0x3f)
                
        t_now = datetime.now()
        heure_utc = datetime.utcnow()

        FS_SEL = 131
        #print("Gyro X: ", gyro_xout, " scale: ", (gyro_xout / FS_SEL))
        #print("Gyro Y: ", gyro_yout, " scale: ", (gyro_yout / FS_SEL))
        #print("Gyro Z: ", gyro_zout, " scale: ", (gyro_zout / FS_SEL))
        gyro_xout = (gyro_xout - base_x_gyro)/FS_SEL
        gyro_yout = (gyro_yout - base_y_gyro)/FS_SEL
        gyro_zout = (gyro_zout - base_z_gyro)/FS_SEL
                
        #print("Gyro Xbase: ", base_x_gyro)
        #print("Gyro Ybase: ", base_y_gyro)
        #print("Gyro Zbase: ", base_z_gyro)
        #print("Gyro X -base: ", gyro_xout)
        #print("Gyro Y -base: ", gyro_yout)
            #print("Gyro Z -base: ", gyro_zout)
            
            # accel_xout_scaled = accel_xout / 16384.0
            # accel_yout_scaled = accel_yout / 16384.0
            # accel_zout_scaled = accel_zout / 16384.0
            
            # print("Accel X: ", accel_xout, " scale: ", accel_xout_scaled)
            # print("Accel Y: ", accel_yout, " scale: ", accel_yout_scaled)
            # print("Accel Z: ", accel_zout, " scale: ", accel_zout_scaled)
                
        RADIANS_TO_DEGREES = 180/math.pi

        accel_angle_y = math.atan(-1*accel_xout/ dist(accel_xout,accel_zout))*RADIANS_TO_DEGREES
        accel_angle_x = math.atan(accel_yout/ dist(accel_yout,accel_zout))*RADIANS_TO_DEGREES
        accel_angle_z = 0
        #    
        #       print("X rotation: " , get_x_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled))
        #        print("Y rotation: " , get_y_rotation(accel_xout_scaled, accel_yout_scaled, accel_zout_scaled))
        
                # Compute the (filtered) gyro angles
        dt = (t_now - last_read_time).microseconds/1000000
        gyro_angle_x = gyro_xout*dt + last_x_angle
        gyro_angle_y = gyro_yout*dt + last_y_angle
        gyro_angle_z = gyro_zout*dt + last_z_angle

                # Compute the drifting gyro angles
        unfiltered_gyro_angle_x = gyro_xout*dt + last_gyro_x_angle
        unfiltered_gyro_angle_y = gyro_yout*dt + last_gyro_y_angle
        unfiltered_gyro_angle_z = gyro_zout*dt + last_gyro_z_angle

                # Apply the complementary filter to figure out the change in angle
                # - choice of alpha is estimated now.
                # Alpha depends on the sampling rate...
        alpha = 0.96
        angle_x = alpha * gyro_angle_x + (1.0 - alpha)*accel_angle_x
        angle_y = alpha * gyro_angle_y + (1.0 - alpha)*accel_angle_y
        angle_z = gyro_angle_z # Accelerometer doesn't given z-angle
                
                # Update the saved data with the lastest values
        set_last_read_angle_data(t_now, angle_x, angle_y, angle_z,\
                gyro_angle_x, gyro_angle_y, gyro_angle_z)
        data = {
            #"delta_time":dt,
            #"gyro_angle_x":gyro_angle_x,
            #"gyro_angle_y":gyro_angle_y,
            #"gyro_angle_z":gyro_angle_z,
            #"accel_angle_x":accel_angle_x,
            #"accel_angle_y":accel_angle_y,
            #"accel_angle_z":accel_angle_z,
            'Roll':angle_x,
            'Pitch':angle_y,
            'Yaw':angle_z
        }
                #print()
        print("---------------------------------------------------------------------")
        print(data,"Heure UTC :", heure_utc)
        
        
        with lcd_lock:
            lcd.message = f"R:{angle_x:.2f} P:{angle_y:.2f}\nY:{angle_z:.2f}"
            sleep(2)
            lcd.clear()
            
    except IOError as e:
        if e.errno == 121:
            print("Remote I/O error occurred. Check your connection.")
            # Tentative de récupération des données en réinitialisant le module
            initial_sensor_module()
            MPU()
        else:
            print("An error occurred: ", e)
                
    except IOError:
        print('An error occured trying to read..(IOError)')
    except KeyboardInterrupt:
        print('The operation have been cancel by keyboard.(KeyboardInterrupt)')


gpsd =gps(mode=WATCH_ENABLE|WATCH_NEWSTYLE)



if __name__ == '__main__':
    initial_sensor_module()
    while True:
        
        thread1 = threading.Thread(target=CM)
        thread2 = threading.Thread(target=MPU)
        thread3 = threading.Thread(target=getPositionData(gpsd))
        
        thread1.start()
        thread2.start()
        thread3.start()
        
        thread1.join()
        thread2.join()
        thread3.join()
        
        
        
     
