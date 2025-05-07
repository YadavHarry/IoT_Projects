import time
import threading
import io
from flask import Flask, render_template, Response, request, jsonify
from flask_socketio import SocketIO
import picamera
import board
import busio
import RPi.GPIO as GPIO
from adafruit_vl53l0x import VL53L0X


def forword():
    GPIO.output(MOTOR_PINS["IN1"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["IN2"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN3"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["IN4"], GPIO.LOW)

def backword():
    GPIO.output(MOTOR_PINS["IN1"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN2"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["IN3"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN4"], GPIO.HIGH)
def left():
    GPIO.output(MOTOR_PINS["IN1"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["IN2"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN3"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN4"], GPIO.HIGH)

def right():
    GPIO.output(MOTOR_PINS["IN1"], GPIO.LOW)
    GPIO.output(MOTOR_PINS["IN2"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["IN3"], GPIO.HIGH)
    GPIO.output(MOTOR_PINS["IN4"], GPIO.LOW)




# Flask and SocketIO setup
app = Flask(__name__)
socketio = SocketIO(app)

# LiDAR and GPIO setup
i2c = busio.I2C(board.SCL, board.SDA)
lidar = VL53L0X(i2c)

# Motor pins
MOTOR_PINS = {
    "IN1": 17,  # GPIO 17
    "IN2": 27,  # GPIO 27
    "IN3": 22,  # GPIO 22
    "IN4": 23,  # GPIO 23
}

# LED pin
LED_PIN = 18  # GPIO 18

# Initialize GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)
for pin in MOTOR_PINS.values():
    GPIO.setup(pin, GPIO.OUT)

# Global variables
distance = "Calculating..."
robot_stopped = False

# Function to update LiDAR distance and control LED/robot
def update_lidar_distance():
    global distance, robot_stopped
    while True:
        try:
            dist = lidar.range
            distance = f"{dist} mm"

            # Logic when distance < 90mm
            if dist < 110:
                GPIO.output(LED_PIN, GPIO.HIGH)  # Turn on LED
                stop_robot()  # Stop the robot
                robot_stopped = True
                message = "Object detected"
                
            else:
                GPIO.output(LED_PIN, GPIO.LOW)  # Turn off LED
                robot_stopped = False
                message = f"Distance: {dist} mm"

            socketio.emit("update_distance", {"distance": message})
        except Exception as e:
            distance = "Error"
        time.sleep(0.5)

# Flask routes
@app.route("/")
def index():
    return render_template("index.html")

@app.route("/video_feed")
def video_feed():
    return Response(gen_frames(), mimetype="multipart/x-mixed-replace; boundary=frame")

@app.route("/control_motor", methods=["POST"])
def control_motor():
    data = request.json
    direction = data.get("direction")
    if robot_stopped:
        return jsonify({"message": "Robot stopped due to obstacle."}), 403

    if direction == "forward":
        GPIO.output(MOTOR_PINS["IN1"], GPIO.HIGH)
        GPIO.output(MOTOR_PINS["IN2"], GPIO.LOW)
        GPIO.output(MOTOR_PINS["IN3"], GPIO.HIGH)
        GPIO.output(MOTOR_PINS["IN4"], GPIO.LOW)
    elif direction == "backward":
        GPIO.output(MOTOR_PINS["IN1"], GPIO.LOW)
        GPIO.output(MOTOR_PINS["IN2"], GPIO.HIGH)
        GPIO.output(MOTOR_PINS["IN3"], GPIO.LOW)
        GPIO.output(MOTOR_PINS["IN4"], GPIO.HIGH)
    elif direction == "left":
        GPIO.output(MOTOR_PINS["IN1"], GPIO.HIGH)
        GPIO.output(MOTOR_PINS["IN2"], GPIO.LOW)
        GPIO.output(MOTOR_PINS["IN3"], GPIO.LOW)
        GPIO.output(MOTOR_PINS["IN4"], GPIO.HIGH)
    elif direction == "right":
        GPIO.output(MOTOR_PINS["IN1"], GPIO.LOW)
        GPIO.output(MOTOR_PINS["IN2"], GPIO.HIGH)
        GPIO.output(MOTOR_PINS["IN3"], GPIO.HIGH)
        GPIO.output(MOTOR_PINS["IN4"], GPIO.LOW)
    else:  # Stop
        stop_robot()

    return jsonify({"message": f"Robot moving {direction}."})

def stop_robot():
    for pin in MOTOR_PINS.values():
        GPIO.output(pin, GPIO.LOW)

def gen_frames():
    with picamera.PiCamera(resolution="640x480", framerate=24) as camera:
        stream = io.BytesIO()
        for _ in camera.capture_continuous(stream, format="jpeg", use_video_port=True):
            stream.seek(0)
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n\r\n" + stream.read() + b"\r\n")
            stream.seek(0)
            stream.truncate()

# Start LiDAR thread
threading.Thread(target=update_lidar_distance, daemon=True).start()

if __name__ == "__main__":
    try:
        socketio.run(app, host="0.0.0.0", port=5000)
    except KeyboardInterrupt:
        GPIO.cleanup()
