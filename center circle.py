import cv2   ###227
import numpy as np
import lgpio
import time
import board
from adafruit_ina219 import INA219



# LCD Pin Configuration
LCD_RS = 26  # Register select
LCD_E = 19   # Enable pin
LCD_D4 = 21  # Data pin 4
LCD_D5 = 20  # Data pin 5
LCD_D6 = 16  # Data pin 6
LCD_D7 = 12  # Data pin 7

# LCD Commands
LCD_CLEAR = 0x01
LCD_HOME = 0x02
LCD_ENTRYMODE = 0x06
LCD_DISPLAYON = 0x0C
LCD_FUNCTIONSET = 0x28

# Initialize GPIO
h = lgpio.gpiochip_open(0)
lgpio.gpio_claim_output(h, LCD_RS)
lgpio.gpio_claim_output(h, LCD_E)
lgpio.gpio_claim_output(h, LCD_D4)
lgpio.gpio_claim_output(h, LCD_D5)
lgpio.gpio_claim_output(h, LCD_D6)
lgpio.gpio_claim_output(h, LCD_D7)

# GPIO Pin Configuration for TB6612FNG Motor Driver
PWMA = 18  # Motor A PWM (X-axis)
AIN1 = 24  # Motor A direction
AIN2 = 23
STBY = 25  # Standby (Enable Motors)
BIN1 = 22  # Motor B direction (Y-axis)
BIN2 = 27
PWMB = 17  # Motor B PWM (Y-axis)

# LED Pin Configuration
LED_PIN = 1  # GPIO1 on Raspberry Pi 5

# Servo Pin Configuration
SERVO_PIN = 13  # GPIO13 (Physical Pin 33)

# Constants for servo control
PWM_FREQUENCY = 50  # Standard 50Hz for servos
PWM_PERIOD_US = 20000  # 20ms in microseconds
PWM_MIN = 500    # 0-degree pulse width in µs (0.5ms)
PWM_MAX = 2500   # 180-degree pulse width in µs (2.5ms)

# Open GPIO chip
chip = lgpio.gpiochip_open(0)

# Set pin modes for motor driver
for pin in [PWMA, AIN1, AIN2, STBY, BIN1, BIN2, PWMB]:
    lgpio.gpio_claim_output(chip, pin)

# Set LED pin as output
lgpio.gpio_claim_output(chip, LED_PIN)

# Set up servo pin
lgpio.gpio_claim_output(chip, SERVO_PIN)

# Enable the motor driver
lgpio.gpio_write(chip, STBY, 1)

# Function to control Motor A (X direction)
def control_motor_A(speed, direction):
    if speed == 0:
        lgpio.tx_pwm(chip, PWMA, 1000, 0)  # Stop motor
    else:
        lgpio.gpio_write(chip, AIN1, 1 if direction == "right" else 0)
        lgpio.gpio_write(chip, AIN2, 0 if direction == "right" else 1)
        lgpio.tx_pwm(chip, PWMA, 1000, speed)

# Function to control Motor B (Y direction)
def control_motor_B(speed, direction):
    if speed == 0:
        lgpio.tx_pwm(chip, PWMB, 1000, 0)  # Stop motor
    else:
        lgpio.gpio_write(chip, BIN1, 1 if direction == "forward" else 0)
        lgpio.gpio_write(chip, BIN2, 0 if direction == "forward" else 1)
        lgpio.tx_pwm(chip, PWMB, 1000, speed)

# Function to control LED
def control_led(state):
    lgpio.gpio_write(chip, LED_PIN, 1 if state else 0)
    print(f"LED turned {'ON' if state else 'OFF'}")

# Function to control servo
def set_servo_angle(angle):
    """Convert angle (0-120) to a valid PWM duty cycle"""
    if 0 <= angle <= 120:  # Restrict angle between 0 and 100 degrees
        pulse_width = ((angle / 180) * (PWM_MAX - PWM_MIN)) + PWM_MIN
        duty_cycle = int((pulse_width / PWM_PERIOD_US) * 100)  # Convert µs to duty cycle
        lgpio.tx_pwm(chip, SERVO_PIN, PWM_FREQUENCY, duty_cycle)
        print(f"Servo set to {angle} degrees")
    else:
        print("Invalid angle! Please enter an angle between 0 and 100 degrees.")

# Function to find the largest detected circle in the frame
def find_circle_center(img):
<<<<<<< HEAD
    small_img = cv2.resize(img, (320, 240))
    gray = cv2.cvtColor(small_img, cv2.COLOR_BGR2GRAY)
    gray = cv2.equalizeHist(gray)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    circles = cv2.HoughCircles(blurred, cv2.HOUGH_GRADIENT, dp=1.2, minDist=50,
                               param1=150, param2=60, minRadius=20, maxRadius=40)
=======

    small_img = cv2.resize(img, (320, 240))


    gray = cv2.cvtColor(small_img, cv2.COLOR_BGR2GRAY)


    thresh = cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                                   cv2.THRESH_BINARY_INV, 11, 2)


    kernel = np.ones((3, 3), np.uint8)
    cleaned = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel, iterations=1)


    blurred = cv2.GaussianBlur(cleaned, (5, 5), 0)


    contours, _ = cv2.findContours(blurred, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    valid_circles = []
    
    for cnt in contours:
        (x, y), radius = cv2.minEnclosingCircle(cnt)
        center = (int(x), int(y))
        radius = int(radius)

        
        perimeter = cv2.arcLength(cnt, True)
        approx_area = cv2.contourArea(cnt)
        circularity = (4 * np.pi * approx_area) / (perimeter ** 2 + 1e-5)

      
        if 0.75 < circularity < 1.2 and 25 < radius < 70:
            valid_circles.append((center[0], center[1], radius))

   
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, dp=1.2, minDist=30,
                               param1=100, param2=50, minRadius=20, maxRadius=70)
>>>>>>> 8f696e671cee8cc2871225ced07b69944b02f321

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        valid_circles.extend(circles)

   
    return max(valid_circles, key=lambda c: c[2]) if valid_circles else None

# Function to calculate displacement from the camera center
def calculate_offset(camera_center, circle_center, tolerance=8):
    x_offset = circle_center[0] - camera_center[0]
    y_offset = circle_center[1] - camera_center[1]

    if abs(x_offset) <= tolerance and abs(y_offset) <= tolerance:
        return (0, 0)  # Stop motors when inside the center region

    return (x_offset, y_offset)

# Function to map displacement to motor speed
def get_motor_speed(offset, max_speed=80):
    min_speed = 50  # Ensure minimum speed is high enough
    scale_factor = 0.8  # Make speed scale more aggressively
    speed = int(min(max(abs(offset) * scale_factor, min_speed), max_speed))
    return speed

#function to go to it's initiaL POSITION
def move_to_initial_charging_position():
    print("Moving to  initial charging position...")
    set_servo_angle(100)
    time.sleep(1)
    # Note: Servo should already be at 100 degrees
    
    # Move backward for alignment on Y-axis
    control_motor_B(81,"forward")
    time.sleep(2)  # Adjust based on required distance
    control_motor_B(0, "forward")  # Stop motor
    
    # Move forward on X-axis for final alignment
    control_motor_A(54, "left")
    time.sleep(1)  # Adjust based on required distance
    control_motor_A(0, "left")  # Stop motor
    
    print("Initialized....")
    
    # Retract servo to 0 degrees after charging position is complete
    
      # Give time for servo to fully retract
    
    # Turn off LED as soon as charging starts
    control_led(False)


# Function to activate servo and move to charging position
def move_to_charging_position():
    print("Moving to charging position...")
    # Note: Servo should already be at 100 degrees
    
    # Move backward for alignment on Y-axis
    control_motor_B(81, "backward")
    time.sleep(2)  # Adjust based on required distance
    control_motor_B(0, "backward")  # Stop motor
    
    # Move forward on X-axis for final alignment
    control_motor_A(54, "right")
    time.sleep(1)  # Adjust based on required distance
    control_motor_A(0, "right")  # Stop motor
    
    print("Charging")
    
    # Retract servo to 0 degrees after charging position is complete
    set_servo_angle(0)
    time.sleep(1)  # Give time for servo to fully retract
    
    # Turn off LED as soon as charging starts
    control_led(False)

# Function to control motors based on the detected circle position
def move_motors(x_move, y_move):
    if x_move == 0 and y_move == 0:
        control_motor_A(0, "right")  # Stop Motor A
        control_motor_B(0, "forward")  # Stop Motor B
        return

    # Control X-axis movement
    if x_move > 0:
        control_motor_A(get_motor_speed(x_move), "right")
    elif x_move < 0:
        control_motor_A(get_motor_speed(x_move), "left")
    
    # Control Y-axis movement
    if y_move > 0:
        control_motor_B(get_motor_speed(y_move), "forward")
    elif y_move < 0:
        control_motor_B(get_motor_speed(y_move), "backward")

# Function to draw crosshair at the camera center
def draw_camera_center(frame, center, crosshair_length=60, thickness=2, radius=5):
    cv2.line(frame, (center[0] - crosshair_length, center[1]), 
             (center[0] + crosshair_length, center[1]), (0, 0, 255), thickness)
    cv2.line(frame, (center[0], center[1] - crosshair_length), 
             (center[0], center[1] + crosshair_length), (0, 0, 255), thickness)
    cv2.circle(frame, center, radius, (0, 0, 255), 2)
    
    


def lcd_pulse_enable():
    lgpio.gpio_write(h, LCD_E, 1)
    time.sleep(0.0005)
    lgpio.gpio_write(h, LCD_E, 0)
    time.sleep(0.0005)

def lcd_send_nibble(data):
    lgpio.gpio_write(h, LCD_D4, (data & 0x01) != 0)
    lgpio.gpio_write(h, LCD_D5, (data & 0x02) != 0)
    lgpio.gpio_write(h, LCD_D6, (data & 0x04) != 0)
    lgpio.gpio_write(h, LCD_D7, (data & 0x08) != 0)
    lcd_pulse_enable()

def lcd_send_byte(data, rs):
    lgpio.gpio_write(h, LCD_RS, rs)
    lcd_send_nibble(data >> 4)  # Send higher nibble
    lcd_send_nibble(data & 0x0F)  # Send lower nibble

def lcd_command(cmd):
    lcd_send_byte(cmd, 0)
    time.sleep(0.002)

def lcd_write_char(char):
    lcd_send_byte(ord(char), 1)

def lcd_write_string(message):
    for char in message:
        lcd_write_char(char)

def lcd_init():
    time.sleep(0.05)
    lcd_send_nibble(0x03)
    time.sleep(0.005)
    lcd_send_nibble(0x03)
    time.sleep(0.0002)
    lcd_send_nibble(0x03)
    lcd_send_nibble(0x02)  # Set to 4-bit mode
    lcd_command(LCD_FUNCTIONSET)
    lcd_command(LCD_DISPLAYON)
    lcd_command(LCD_CLEAR)
    lcd_command(LCD_ENTRYMODE)

def lcd_clear():
    lcd_command(LCD_CLEAR)
    time.sleep(0.002)

def lcd_set_cursor(line, pos):
    address = 0x80 + pos if line == 0 else 0xC0 + pos
    lcd_command(address)
    
    
# Initialize LCD
lcd_init()

# Setup I2C bus for INA219
i2c_bus = board.I2C()

# Initialize INA219 sensors with different addresses
ina1 = INA219(i2c_bus, addr=0x40)
ina2 = INA219(i2c_bus, addr=0x41)

def run_monitoring():
    """Main function to monitor and display INA219 sensor data"""
    try:
        # Initialize LCD
        lcd_init()
        
        # Setup I2C bus for INA219
        i2c_bus = board.I2C()
        
        # Initialize INA219 sensors with different addresses
        ina1 = INA219(i2c_bus, addr=0x40)
        ina2 = INA219(i2c_bus, addr=0x41)
        
        while True:
            # Read Sensor Data
            voltage1 = ina1.bus_voltage
            current1 = ina1.current
            voltage2 = ina2.bus_voltage
            current2 = ina2.current
            
            # Format Data for LCD
            line1 = f"V1:{voltage1:.2f}V I1:{current1:.2f}mA"
            line2 = f"V2:{voltage2:.2f}V I2:{current2:.2f}mA"
            
            # Display Data on LCD
            lcd_clear()
            lcd_set_cursor(0, 0)
            lcd_write_string(line1[:16])  # Ensure only 16 chars max per line
            lcd_set_cursor(1, 0)
            lcd_write_string(line2[:16])  # Limit to 16 characters
            
            # Print Data to Terminal (Optional)
            print(f"INA219 (0x40) Voltage: {voltage1:.2f}V, Current: {current1:.2f}mA")
            print(f"INA219 (0x41) Voltage: {voltage2:.2f}V, Current: {current2:.2f}mA")
            print("-" * 40)
            
            # Wait before next update
            time.sleep(2)
            
    except KeyboardInterrupt:
        # Handle Ctrl+C gracefully
        print("Keyboard interrupt detected.")
    except Exception as e:
        # Handle other exceptions
        print(f"Error occurred: {e}")
    finally:
        # Always run cleanup
        cleanup()



# Main function
def main():
    # Initialize servo to 100 degrees at system start
    set_servo_angle(120)
    time.sleep(1)
    
    cap = cv2.VideoCapture(0)
<<<<<<< HEAD
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    # Flag to track if we've already found the center and moved to charging position
    center_found = False
    led_is_on = False
    
    # Turn on LED when alignment starts
    control_led(True)
    led_is_on = True

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            height, width, _ = frame.shape
            camera_center = (width // 2, height // 2)

            draw_camera_center(frame, camera_center)

            circle = find_circle_center(frame)
            if circle is not None:
                x, y, r = circle
                circle_center = (x * 2, y * 2)  # Scale back to original frame size
                x_move, y_move = calculate_offset(camera_center, circle_center)
=======

   
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        height, width, _ = frame.shape
        camera_center = (width // 2, height // 2)
        camera_center_radius = 10  

        cv2.circle(frame, camera_center, camera_center_radius, (0, 0, 255), -1)

        circle = find_circle_center(frame)
        if circle is not None:
            x, y, r = circle
            circle_center = (x * 2, y * 2)  
            distance = calculate_distance(camera_center, circle_center)
            x_move = circle_center[0] - camera_center[0]
            y_move = circle_center[1] - camera_center[1]


            cv2.circle(frame, circle_center, r * 2, (0, 255, 0), 2)
            cv2.circle(frame, circle_center, 5, (0, 255, 0), -1)
>>>>>>> 8f696e671cee8cc2871225ced07b69944b02f321

                # Check if we're at the center and haven't already processed this
                if x_move == 0 and y_move == 0 and not center_found:
                    
                    print("Center found! Stopping alignment.")
                 
                    move_motors(0, 0)  # Stop motors
                    center_found = True
                    
                    # Add text to frame
                    cv2.putText(frame, "CENTER FOUND!", (width//2-100, height//2-50), 
                                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    
                    # Show the frame with the center found message
                    cv2.imshow('frame', frame)
                    cv2.waitKey(1)
                    
                    # Move to charging position and then set servo to 0 degrees
                    move_to_charging_position()
                    run_monitoring()
                    #led_is_on = False
                    # Ensure LED is off
                    lgpio.gpio_write(chip, LED_PIN, 0)
                elif not center_found:
                    move_motors(x_move, y_move)  # Control motors
                    # Ensure LED is on during alignment
                    if not led_is_on:
                        control_led(True)
                        led_is_on = True
                
                cv2.circle(frame, circle_center, r * 2, (0, 255, 0), 2)
                cv2.circle(frame, circle_center, 5, (0, 255, 0), -1)

<<<<<<< HEAD
                text_move = f"Move: ({x_move}, {y_move})"
                
                # Add status text
                if center_found:
                    cv2.putText(frame, "CHARGING", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                text_move = "Move: (N/A, N/A)"
                move_motors(0, 0)  # Stop motors if no circle detected
                
                # If we lost the circle but had previously found it
                if center_found:
                    center_found = False
                    # Turn LED back on if we need to realign
                    if not led_is_on:
                        control_led(True)
                        led_is_on = True
                    # Keep servo at 100 degrees - don't change it
                
                # Make sure LED is on during alignment if it's not already on
                if not led_is_on:
                    control_led(True)
                    led_is_on = True

            cv2.putText(frame, text_move, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            
            # Add LED and servo status to frame
            led_status = "LED: ON" if led_is_on else "LED: OFF"
            servo_angle = 0 if center_found and not led_is_on else 100  # 0 only after charging complete
            servo_status = f"SERVO: {servo_angle} DEGREES"
            cv2.putText(frame, led_status, (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
            cv2.putText(frame, servo_status, (10, 120), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 255), 2)
            
            cv2.imshow('frame', frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Program interrupted by user")
    finally:
        # Clean up
        cap.release()
        move_to_initial_charging_position()
        cv2.destroyAllWindows()
        # Return servo to initial position
        set_servo_angle(120)
        time.sleep(1)
        # Stop PWM for servo
        lgpio.tx_pwm(chip, SERVO_PIN, PWM_FREQUENCY, 0)
        # Disable motor driver
        lgpio.gpio_write(chip, STBY, 0)
        # Ensure LED is off
        lgpio.gpio_write(chip, LED_PIN, 0)
        # Close GPIO chip
        lgpio.gpiochip_close(chip)
        
=======

        cv2.putText(frame, text_distance, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        cv2.putText(frame, text_move, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
>>>>>>> 8f696e671cee8cc2871225ced07b69944b02f321




if __name__ == "__main__":
    main()
    
    
    
#python3 -c "import lgpio; h = lgpio.gpiochip_open(0); lgpio.gpiochip_close(h)"

