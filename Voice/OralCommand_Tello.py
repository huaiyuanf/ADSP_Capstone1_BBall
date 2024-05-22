from djitellopy import Tello
import time
import speech_recognition as sr

# Create a Tello object
tello = Tello('192.168.1.79')

# Function to listen and recognize speech
def recognize_speech_from_mic(recognizer, microphone):
    # Check that recognizer and microphone arguments are appropriate type
    if not isinstance(recognizer, sr.Recognizer):
        raise TypeError("`recognizer` must be `Recognizer` instance")
    if not isinstance(microphone, sr.Microphone):
        raise TypeError("`microphone` must be `Microphone` instance")

    # Adjust the recognizer sensitivity to ambient noise and record audio from the microphone
    with microphone as source:
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source)

    # Set up the response object
    response = {
        "success": True,
        "error": None,
        "transcription": None
    }

    try:
        response["transcription"] = recognizer.recognize_google(audio)
    except sr.RequestError:
        response["success"] = False
        response["error"] = "API unavailable"
    except sr.UnknownValueError:
        response["error"] = "Unable to recognize speech"

    return response

# Create recognizer and microphone instances
recognizer = sr.Recognizer()
microphone = sr.Microphone()

# Try to connect to the drone
try:
    print("Connecting to drone...")
    tello.connect()
    print("Connected to drone!")
    print("Battery level: %s" % tello.get_battery())
    last_command_time = time.time()
    while True:
        print("Listening for commands. Say 'apple' to take off or 'car' to land")
        command = recognize_speech_from_mic(recognizer, microphone)
        current_time = time.time()
        if command["transcription"]:
            print("You said: {}".format(command["transcription"]))
        else:
            print("I didn't catch that. What did you say?")
            continue

        transcription = command["transcription"].lower()
        if command["transcription"].lower() == "apple": #takeoff
            print("Taking off!")
            tello.takeoff()
            time.sleep(5)
        elif command["transcription"].lower() == "peach": #land
            print("Landing!")
            tello.land()
            time.sleep(5)
        elif command["transcription"].lower() == "forward":
            print("Moving forward!")
            tello.move_forward(30)  # move forward 30 cm
            time.sleep(5)
        elif command["transcription"].lower() == "backward":
            print("Moving backward!")
            tello.move_back(30)  # move backward 30 cm
            time.sleep(5)
        elif command["transcription"].lower() == "up":
            print("Moving up!")
            tello.move_up(30)  # move up 30 cm
            time.sleep(5)
        elif command["transcription"].lower() == "down":
            print("Moving down!")
            tello.move_down(30)  # move down 30 cm
            time.sleep(5)
        elif command["transcription"].lower() == "left":
            print("Moving left!")
            tello.move_left(30)  # move left 30 cm
            time.sleep(5)
        elif command["transcription"].lower() == "right":
            print("Moving right!")
            tello.move_right(30)  # move right 30 cm
            time.sleep(5)
        elif command["transcription"].lower() == "flip left": # flip left
            print("Flipping left!")
            tello.flip_left()
            time.sleep(5)
        elif command["transcription"].lower() == "flip right": # flip right
            print("Flipping right!")
            tello.flip_right()
            time.sleep(5)
        elif command["transcription"].lower() == "flip forward": # flip foward
            print("Flipping forward!")
            tello.flip_forward()
            time.sleep(5)
        elif command["transcription"].lower() == "flip backward": # flip backward
            print("Flipping backward!")
            tello.flip_back()
            time.sleep(5)
        elif command["transcription"].lower() == "rotate left": 
            print("Rotating left!")
            tello.rotate_counter_clockwise(90)  # rotate left 90 degrees
            time.sleep(5)
        elif command["transcription"].lower() == "rotate right":
            print("Rotating right!")
            tello.rotate_clockwise(90)  # rotate right 90 degrees
            time.sleep(5)
        else:
            print("Invalid command!")
            
        if current_time - last_command_time > 40:
            print("No command received for over 40 seconds. Landing the drone.")
            tello.land()
            break
except Exception as e:
    print(f"Failed to connect to the drone: {e}")
