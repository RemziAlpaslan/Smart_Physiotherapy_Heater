import RPi.GPIO as GPIO
import smbus
import time
import os
import glob

# GPIO setup for keypad
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

# Pin definitions for rows and columns
ROW_PINS = [19, 23, 24, 25]
COL_PINS = [26, 17, 27, 22]

# GPIO pinlerinin ayarlanması
GPIO.setmode(GPIO.BCM)
TRIG = 16
ECHO = 12

GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# GPIO ayarlarını yap
GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)


# PWM sinyalini ayarla
pwm = GPIO.PWM(18, 50)  # 50Hz PWM frekansı
pwm.start(0)

#these tow lines mount the device:
os.system('modprobe w1-gpio')
os.system('modprobe w1-therm')
 
base_dir = '/sys/bus/w1/devices/'
device_path = glob.glob(base_dir + '28*')[0] #get file path of sensor
rom = device_path.split('/')[-1] #get rom name

def convert_seconds_to_min_sec(seconds):
    minutes = seconds // 60
    seconds = seconds % 60
    return f"{minutes:02}:{seconds:02}"


class PID:
    def __init__(self, Kp, Ki, Kd, setpoint, sample_time=0.01):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.sample_time = sample_time

        self.clear()

    def clear(self):
        """Reset PID computations and coefficients"""
        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value):
        """Calculate PID output value for given reference feedback"""
        error = self.setpoint - feedback_value

        self.PTerm = self.Kp * error
        self.ITerm += error * self.sample_time

        if self.ITerm < -self.windup_guard:
            self.ITerm = -self.windup_guard
        elif self.ITerm > self.windup_guard:
            self.ITerm = self.windup_guard

        self.DTerm = (error - self.last_error) / self.sample_time

        # Remember last error for next calculation
        self.last_error = error

        # Calculate PID output
        self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)



def read_temp_raw():
    with open(device_path +'/w1_slave','r') as f:
        valid, temp = f.readlines()
    return valid, temp
 
def read_temp():
    valid, temp = read_temp_raw()

    while 'YES' not in valid:
        time.sleep(0.2)
        valid, temp = read_temp_raw()

    pos = temp.index('t=')
    if pos != -1:
        #read the temperature .
        temp_string = temp[pos+2:]
        temp_c = float(temp_string)/1000.0 
        temp_f = temp_c * (9.0 / 5.0) + 32.0
        return temp_c, temp_f
 
print(' ROM: '+ rom)

# Keypad layout
KEYPAD = [
    [1, 2, 3, 'A'],
    [4, 5, 6, 'B'],
    [7, 8, 9, 'C'],
    ['*', 0, '#', 'D']
]

# Setup row pins as outputs
for pin in ROW_PINS:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

# Setup column pins as inputs with pull-down resistor
for pin in COL_PINS:
    GPIO.setup(pin, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# I2C LCD setup
I2C_BUS = 1
I2C_ADDR = 0x27
LCD_CHR = 1
LCD_CMD = 0
LCD_BACKLIGHT = 0x08
ENABLE = 0b00000100
LCD_WIDTH = 20
LCD_LINES = [0x80, 0xC0, 0x94, 0xD4]
bus = smbus.SMBus(I2C_BUS)

def lcd_init():
    lcd_byte(0x33, LCD_CMD)
    lcd_byte(0x32, LCD_CMD)
    lcd_byte(0x06, LCD_CMD)
    lcd_byte(0x0C, LCD_CMD)
    lcd_byte(0x28, LCD_CMD)
    lcd_byte(0x01, LCD_CMD)
    time.sleep(0.0005)

def lcd_byte(bits, mode):
    bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT
    bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT
    bus.write_byte(I2C_ADDR, bits_high)
    lcd_toggle_enable(bits_high)
    bus.write_byte(I2C_ADDR, bits_low)
    lcd_toggle_enable(bits_low)

def lcd_toggle_enable(bits):
    time.sleep(0.0005)
    bus.write_byte(I2C_ADDR, (bits | ENABLE))
    time.sleep(0.0005)
    bus.write_byte(I2C_ADDR, (bits & ~ENABLE))
    time.sleep(0.0005)

def lcd_string(message, line, col, delet):
    address = LCD_LINES[line] + col
    lcd_byte(address, LCD_CMD)
    for char in message.ljust(delet, " "):
        lcd_byte(ord(char), LCD_CHR)

def read_keypad():
    pressed_key = None
    for row_num, row_pin in enumerate(ROW_PINS):
        GPIO.output(row_pin, GPIO.HIGH)
        for col_num, col_pin in enumerate(COL_PINS):
            if GPIO.input(col_pin) == GPIO.HIGH:
                pressed_key = KEYPAD[row_num][col_num]
                while GPIO.input(col_pin) == GPIO.HIGH:
                    time.sleep(0.01)
        GPIO.output(row_pin, GPIO.LOW)
    return pressed_key





def measure_distance():
    sayac = 0
    
    # Trig pinini düşük yaparak sensörü temizle
    GPIO.output(TRIG, False)
    time.sleep(0.002)  # Sensörü temizlemek için bekleyin
    
    # Trig pinini yüksek yaparak ölçüm başlat
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    
    # Echo pininden sinyalin geri dönüş süresini ölç
    pulse_start = time.time()
    timeout_start = time.time()
    
    while GPIO.input(ECHO) == 0:
        if time.time() - timeout_start > 0.02:  # 20ms zaman aşımı
            return None  # Ölçüm başarısız
        pulse_start = time.time()
    
    pulse_end = time.time()
    timeout_start = time.time()
    
    while GPIO.input(ECHO) == 1:
        if time.time() - timeout_start > 0.02:  # 20ms zaman aşımı
            return None  # Ölçüm başarısız
        pulse_end = time.time()
    
    pulse_duration = pulse_end - pulse_start
    
    # Mesafeyi hesapla (34300 cm/s hızını kullanarak)
    distance = pulse_duration * 17150
    distance = round(distance, 2)
    
    return distance

def set_angle(angle):
    duty = angle / 18 + 2
    GPIO.output(18, True)
    pwm.ChangeDutyCycle(duty)
    time.sleep(0.8)
    GPIO.output(18, False)
    pwm.ChangeDutyCycle(0)



lcd_init()
lcd_string("Ayar Sicaklik:", 0, 0,len("Ayar Sicaklik:"))
lcd_string("Ayar Sure:", 1, 0,len("Ayar Sure:"))
lcd_string("Sic.:", 2, 0,len("Sic.:"))
lcd_string("Sur.:", 3, 0,len("Sur.:"))
lcd_string("Uzk:", 2, 12,len("Uzk:"))




input_buffer = ""
current_input_line = None

başlat=False
ayarSıcaklığı=45
ayarSüres=15
sıckaklık=0
süre=0
uzaklık=0
timer=0
temp=0
angle=90
set_angle(0)

Kp = 4.0
Ki = 3.0
Kd = 1.0
pid = PID(Kp, Ki, Kd, ayarSıcaklığı,1)



try:
    while True:
        temp+=1            
        # burası lcd ile alakalı yer
        if başlat:
            lcd_string("CLS", 3, 12, 3)
        else:
            lcd_string("DUR", 3, 12, 3)            
        lcd_string(str(ayarSıcaklığı), 0, 15,4)
        lcd_string(str(ayarSüres), 1, 15,4)
        lcd_string(str(uzaklık), 2, 17,3)
        lcd_string(str(sıckaklık), 2, 6,4)
        
        

        
            
           
        # burası keypad ile alakalı
        key = read_keypad()
        if key is not None:
            print(key)
            if key == 'D':
                    if başlat==False:
                        # Write to line 3, column 14
                        başlat=True
                    elif başlat==True:
                        # Write to line 3, column 14
                        başlat=False
                        set_angle(0)
            if başlat==False:
                if key == '#':
                    current_input_line = 0  # Update Ayar Sic.
                    input_buffer = ""
                    timer=0
                elif key == '*':
                    current_input_line = 1  # Update Ayar Sur.
                    input_buffer = ""
                elif isinstance(key, (int, float)):
                    # Append numeric input
                    if len(input_buffer) < 2:
                        input_buffer += str(key)
                        # Update the display with the current buffer
                        if current_input_line:
                            ayarSıcaklığı=int(input_buffer)
                            pid = PID(Kp, Ki, Kd, ayarSıcaklığı,1)
                        else:
                            ayarSüres=int(input_buffer)
                            minutes = ayarSüres
                            seconds = minutes * 60
                            
                elif key == 'A':
                    ayarSıcaklığı=45
                    ayarSüres=55
                    
                elif key == 'B':
                    ayarSıcaklığı=50
                    ayarSüres=60
                    
                elif key == 'C':
                    ayarSıcaklığı=35
                    ayarSüres=20
                
                           
        

        # burası ana kod 

        
        if temp > 10:
            temp=0
            dist = measure_distance()
            if dist is not None:
                uzaklık = int(dist)
                print(f"Uzaklık: {uzaklık} cm")
            else:
                print("Ölçüm başarısız")
            sıckaklık=0
            
            
            


            
        
        
        if başlat:
            dist = measure_distance()
            if dist is not None:
                uzaklık = int(dist)
                print(f"Uzaklık: {uzaklık} cm")
            else:
                print("Ölçüm başarısız")
            c, f = read_temp()
            print('C={:,.3f} F={:,.3f}'.format(c, f))
            sıckaklık=round(c,1)
            pid.update(sıckaklık)
            heater_power = pid.output
            süre=ayarSüres*60-timer
            timer+=1
            if heater_power>180:
                heater_power=180
            if heater_power<0:
                heater_power=0
            print(round(heater_power))
            set_angle(round(heater_power))
            lcd_string(convert_seconds_to_min_sec(süre),3,6,3)
            if süre<=0:
                başlat=False
                timer=0
                set_angle(0)

                
                

                        
            
        
except KeyboardInterrupt:
    print("Exiting program.")
finally:
    GPIO.cleanup()
