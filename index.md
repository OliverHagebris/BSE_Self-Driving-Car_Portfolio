# Self-Driving Car
My project is a self-driving car. When it is powered on, it begins its trek, slowly moving forward until it reaches an obstacle, either veering off or reversing to avoid a collision. It runs on an Arduino R3 Uno board, using C++ code, and the entire project uses a 9V battery for power.

<!--Replace this text with a brief description (2-3 sentences) of your project. This description should draw the reader in and make them interested in what you've built. You can include what the biggest challenges, takeaways, and triumphs from completing the project were. As you complete your portfolio, remember your audience is less familiar than you are with all that your project entails!-->
<!--The biggest challenge I had was the wiring; this project had a decently compact wiring setup, which had some columns full of wires.-->


| **Engineer** | **School** | **Area of Interest** | **Grade** |
|:--:|:--:|:--:|:--:|
| Oliver H | Lynbrook Highschool | Electrical Engineering/Mechanical Engineering | Incoming Sophomore

<!--**Replace the BlueStamp logo below with an image of yourself and your completed project. Follow the guide [here](https://tomcam.github.io/least-github-pages/adding-images-github-pages-site.html) if you need help.**-->

![Headstone Image]()
  
# Final Milestone

<!--**Don't forget to replace the text below with the embedding for your milestone video. Go to Youtube, click Share -> Embed, and copy and paste the code to replace what's below.**-->

<iframe width="560" height="315" src="https://www.youtube.com/embed/F7M7imOVGug" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

<!--For your final milestone, explain the outcome of your project. Key details to include are:
- What you've accomplished since your previous milestone
- What your biggest challenges and triumphs were at BSE
- A summary of key topics you learned about
- What you hope to learn in the future after everything you've learned at BSE-->



# Second Milestone

<!--**Don't forget to replace the text below with the embedding for your milestone video. Go to Youtube, click Share -> Embed, and copy and paste the code to replace what's below.**-->

<iframe width="560" height="315" src="https://www.youtube.com/embed/y3VAmNlER5Y" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" allowfullscreen></iframe>

<!--For your second milestone, explain what you've worked on since your previous milestone. You can highlight:
- Technical details of what you've accomplished and how they contribute to the final goal
- What has been surprising about the project so far
- Previous challenges you faced that you overcame
- What needs to be completed before your final milestone -->

# First Milestone


<iframe width="560" height="315" src="https://www.youtube.com/embed/Gh5CL6sdwgA?si=XHWO7IYK0BkYQ-xn" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>
For my first milestone my project is in it's base form; it has 1 9V battery to power the entire project, 2 ir sensors for detecting objects, 1 ultrasound sensor for determining distance to nearest object, 1 arduino uno as its computer, 1 l9110 Module for controlling the motors, 2 TT motors to rotate the wheels, 2 TT wheels to apply the rotational power from the motor as translational power, and 1 universal wheel to support the project and keep it balanced. A challenge I'm facing is turning off the robot as when I need to power it off, it constantly tries to continue driving forward.

<!-- This text is hidden and will not be displayed on the page -->
<!--For your first milestone, describe what your project is and how you plan to build it. You can include:-->
<!--- An explanation about the different components of your project and how they will all integrate together-->
<!--- Technical progress you've made so far-->
<!--- Challenges you're facing and solving in your future milestones-->
<!--- What your plan is to complete your project-->

# Schematics 
<!--Here's where you'll put images of your schematics. [Tinkercad](https://www.tinkercad.com/blog/official-guide-to-tinkercad-circuits) and [Fritzing](https://fritzing.org/learning/) are both great resoruces to create professional schematic diagrams, though BSE recommends Tinkercad becuase it can be done easily and for free in the browser. -->

# Code
```c++
#include <EEPROM.h>
#include <IRremote.h>

const float leftOffset = 0.91; //the offsets of the motors
const float rightOffset = 1.0;

const int A_1B = 5; //the pinholes for each motor and the function
const int A_1A = 6;
const int B_1B = 9;
const int B_1A = 10;

const int echoPin = 4;  //the pinholes for the ultrasound sensor
const int trigPin = 3;

const int rightIR = 7;  //the pinholes for the ir sensor
const int leftIR = 8;

const int IR_RECEIVE_PIN = 12;  //the pinhole for the irremote sensor

bool isOn = false;  //the var that turns on/off the system

float readSensorData() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  float distance = pulseIn(echoPin, HIGH) / 58.00; //Equivalent to (340m/s*1us)/2
  return distance;
}


void moveForward(int speed) {
  analogWrite(A_1B, 0);
  analogWrite(A_1A, (int)(speed*rightOffset));
  analogWrite(B_1B, (int)(speed*leftOffset));
  analogWrite(B_1A, 0);
}

void moveBackward(int speed) {
  analogWrite(A_1B, (int)(speed*rightOffset));
  analogWrite(A_1A, 0);
  analogWrite(B_1B, 0);
  analogWrite(B_1A, (int)(speed*leftOffset));
}


void backLeft(int speed) {
  analogWrite(A_1B, (int)(speed*rightOffset));
  analogWrite(A_1A, 0);
  analogWrite(B_1B, 0);
  analogWrite(B_1A, 0);
}

void backRight(int speed) {
  analogWrite(A_1B, 0);
  analogWrite(A_1A, 0);
  analogWrite(B_1B, 0);
  analogWrite(B_1A, (int)(speed*leftOffset));
}

void stopMove() {
  analogWrite(A_1B, 0);
  analogWrite(A_1A, 0);
  analogWrite(B_1B, 0);
  analogWrite(B_1A, 0);
}

String decodeKeyValue(long result)  //translates the signal from the IR remote into its respective button
{
  switch(result){
    case 0x16:
      return "0";
    case 0xC:
      return "1"; 
    case 0x18:
      return "2"; 
    case 0x5E:
      return "3"; 
    case 0x8:
      return "4"; 
    case 0x1C:
      return "5"; 
    case 0x5A:
      return "6"; 
    case 0x42:
      return "7"; 
    case 0x52:
      return "8"; 
    case 0x4A:
      return "9"; 
    case 0x9:
      return "+"; 
    case 0x15:
      return "-"; 
    case 0x7:
      return "EQ"; 
    case 0xD:
      return "U/SD";
    case 0x19:
      return "CYCLE";         
    case 0x44:
      return "PLAY/PAUSE";   
    case 0x43:
      return "FORWARD";   
    case 0x40:
      return "BACKWARD";   
    case 0x45:
      return "POWER";   
    case 0x47:
      return "MUTE";   
    case 0x46:
      return "MODE";       
    case 0x0:
      return "ERROR";   
    default :
      return "ERROR";
    }
}
void setup() {
  Serial.begin(9600);

  //motor
  pinMode(A_1B, OUTPUT);
  pinMode(A_1A, OUTPUT);
  pinMode(B_1B, OUTPUT);
  pinMode(B_1A, OUTPUT);

  //ultrasonic
  pinMode(echoPin, INPUT);
  pinMode(trigPin, OUTPUT);

  //IR obstacle
  pinMode(leftIR, INPUT);
  pinMode(rightIR, INPUT);

  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);

}

void loop() {

  int left = digitalRead(leftIR);  // 0: Obstructed   1: Empty
  int right = digitalRead(rightIR);
  if (isOn) {

    if (!left && right) {
      backLeft(150);
    } else if (left && !right) {
      backRight(150);
    } else if (!left && !right) {
      moveBackward(150);
    } else {
      float distance = readSensorData();
      Serial.println(distance);
      if (distance > 50) { // Safe
        moveForward(200); //200 is the max rpm that the motor can run at
      } else if (distance < 10) { // Attention
        moveBackward(200);
       delay(1000);
       backLeft(150);
       delay(500);
     } else {
       moveForward(150);
      }
   }
  } else {
    stopMove();
  }
  if (IrReceiver.decode()) {
    //    Serial.println(results.value,HEX);
    String key = decodeKeyValue(IrReceiver.decodedIRData.command);
    if (key != "ERROR") {
      Serial.println(key);

      if (key == "POWER") {
        isOn = !isOn;
        delay(100);
      }
    }
    IrReceiver.resume();
  }
}
```


# Bill of Materials
<!--Here's where you'll list the parts in your project. To add more rows, just copy and paste the example rows below.
Don't forget to place the link of where to buy each component inside the quotation marks in the corresponding row after href =. Follow the guide [here]([url](https://www.markdownguide.org/extended-syntax/)) to learn how to customize this to your project needs. -->

| **Part** | **Note** | **Price** | **Link** |
|:--:|:--:|:--:|:--:|
| 3 in 1 kit | The base project + IRremote modification | $69.99 | <a href="https://www.sunfounder.com/products/sunfounder-3-in-1-iot-smart-car-learning-ultimate-starter-kit"> Link </a> |
| screw terminal DC barrel adapter | What the item is used for | $3.97 | <a href="https://www.amazon.com/dp/B0CR8TZ41W"> Link </a> |
| 5W 12V Solar Panel | Used to power the car | $13.99 | <a href="https://www.amazon.com/Efficiency-Chargerfor-Monocrystalline-Photovoltaic-Batteries/dp/B0F8Q3FTLT"> Link </a> |
| M3x50mm Standoff x8 | Used to elevate the solarpanel above the robot | $9.86 | <a href="https://www.mouser.com/ProductDetail/Davies-Molding/SH1000-K?qs=vLWxofP3U2ymAINUbdVfLQ%3D%3D"> Link </a> |
| M3 Nuts x2 | Used to attach the standoffs to the base project | $0.36 | <a href="https://www.mouser.com/ProductDetail/Essentra/04M030050HNDIN34814?qs=T3oQrply3y%252BoX1ymaXFOZA%3D%3D"> Link </a> |
| J-B Weld | Used to weld together the solar panel and the standoffs | $13.61 | <a href="https://www.amazon.com/J-B-Weld-KwikWeld-Waterproof-50176-2/dp/B009EU5ZMA"> Link </a> |

# Other Resources/Examples
One of the best parts about Github is that you can view how other people set up their own work. Here are some past BSE portfolios that are awesome examples. You can view how they set up their portfolio, and you can view their index.md files to understand how they implemented different portfolio components.
- [Example 1](https://trashytuber.github.io/YimingJiaBlueStamp/)
- [Example 2](https://sviatil0.github.io/Sviatoslav_BSE/)
- [Example 3](https://arneshkumar.github.io/arneshbluestamp/)

To watch the BSE tutorial on how to create a portfolio, click here.
