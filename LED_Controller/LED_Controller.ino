//FRC 2024 LED controller, team 6443 AEMBOT, written by c2bVids

/*
~Version History~
 V1 - Added main functions
 V2 (not on github) - Added more comments, version history, and off button support
 V3 - Added 'pulse' function (I got bored)
*/

/*
Possible CHARACTERS that can be sent:

~Alliance Colors~
 - red (r)
 - blue (b)

~Indicator Colors~
 - orange (o)
 - green (g)

~Fun Patterns~
 - hue fade (f)
 - rainbow (w)

~Other Colors~
 - off (0)
 - yellow (y)
 - purple (p)
 - AEMLIGHT (l)
 - AEMDARK (d)

~Speeds~
 - auto/ teleop speed (1)
 - endgame speed (2)

Send characters through the serial port at 115200 baud, not too fast though please don't make anyone have a siezure
*/

#include <Adafruit_NeoPixel.h>
//include the NeoPixel library for setting the LED's color

#define UnderglowPin 2
//define which pin controls the Underglow LEDs
#define UnderglowLength 58
//define how long the Underglow LED strip is

#define MountedPin 3
//define which pin controls the Mounted LEDs
#define MountedLength 58
//define how long the Mounted LED strip is

Adafruit_NeoPixel Underglow = Adafruit_NeoPixel(UnderglowLength, UnderglowPin, NEO_GRB + NEO_KHZ800);
//declare the Underglow strip
Adafruit_NeoPixel Mounted = Adafruit_NeoPixel(MountedLength, MountedPin, NEO_GRB + NEO_KHZ800);
//declare the Mounted strip

#define ButtonPin 4
//which pin the button is connected to

int shift = 0;
//the value that offsets the patterns during animation

int setLength = 10;

//how long each repeating section of pattern is

int speed = 45;
//how fast the animation moves, the delay between 'frames' in milliseconds

int red = 0;
//declare the initial stored red value
int green = 0;
//declare the initial stored green value
int blue = 0;
//declare the initial stored blue value

void setup() { //setup function, to run once then jump to the 'loop' function

  Underglow.begin();
  //initialize the Underglow LED strip
  Mounted.begin();
  //initialiaze the Mounted LED strip

  Serial.begin(115200);
  //initialize Serial communication at a baud rate of 115200

  pinMode(UnderglowPin, OUTPUT);
  //declare the pin that controls the Underglow LED strip as an output pin
  pinMode(MountedPin, OUTPUT);
  //declare the pin that controls the Mounted LED strip as an output pin
  pinMode(ButtonPin, INPUT);
  //declare the pin that reads the button as an input pin

  turnOn(41, 20, 60, 112, 203);
  //run a function to turn on both strips of LEDs in a cool pattern with color (60, 112, 203), also known as AEMlight

  while (red == 0 && green == 0 && blue == 0) { //while loop to constantly wait for an input from the Serial port

    getColor();
    //gets a viable color, speed, or function sent via the Serial port

    if (digitalRead(ButtonPin) != HIGH) { //off button

      Underglow.clear();
      //off
      Underglow.show();
      //off
      Mounted.clear();
      //off
      Mounted.show();
      //off?

    }

  }

}

float modulo(float a, int n) { //modulo function with support for a float input, and no < 0 outputs

  float mod = fmod(a, n);
  //initial modulo function

  if (mod < 0) { //checking if the result is below 0

    mod += n;
    //if the result is below 0, the second number is added to the result

  }

  return mod;
  //returns the final modulo value

}

void loop() { //loop function, to run indefinitally unless told otherwise

  while (digitalRead(ButtonPin) == HIGH) { //while not off

    getColor();
    //gets a viable color, speed, or function sent via the Serial port

    if (red != 0 || green != 0 || blue != 0) { //check if the current color isn't 'off'

      sawtoothFade(red, green, blue);
      //makes a sawtooth pattern on the LED strips

      shift = modulo(shift+-1, setLength);
      //increments the shift variable, moving the animation, but doesn't let the variable go over the pattern length to save memory

      delay(speed);
      //delay so the shift between frames isn't instant
    }

  }

  while(1) { //off button

    Underglow.clear();
    //off
    Underglow.show();
    //off?
    Mounted.clear();
    //off
    Mounted.show();
    //off!

  }

}

int turnOn(int UnderglowSpeed, int MountedSpeed, int red, int green, int blue) { //function to turn on the LED strips, but *cool*

  for (int i=0; i<UnderglowLength; i++) { //loops through all of the pixel indexes on the Underglow LED strips

    Underglow.setPixelColor(i, Underglow.Color(red, green, blue));
    //queues the given color to a pixel at a given index on the Underglow LED strip

    Underglow.show();
    //show any queued LED colors

    delay(UnderglowSpeed);
    //delay so the strip doesn't just light up all at once

    if (digitalRead(ButtonPin) != HIGH) {
      Underglow.clear();
      Underglow.show();
      return 0;
    }

  }

  delay(500);
  //delay for effect

  for (int i=0; i<MountedLength; i++) { //loops through all of the pixel indexes on the Mounted LED strips

    Mounted.setPixelColor(i, Mounted.Color(red, green, blue));
    //queues the given color to a pixel at a given index on the Mounted LED strip

    Mounted.show();
    //show any queued LED colors

    delay(MountedSpeed);
    //delay so the strip doesn't just light up all at once

    if (digitalRead(ButtonPin) != HIGH) {
      Underglow.clear();
      Underglow.show();
      Mounted.clear();
      Mounted.show();
      return 0;
    }

  }

  return 0;

}

void getColor() { //gets a viable color, speed, or function from the Serial port

  if (Serial.available()) { //checks if anything has been sent via the Serial port

    int input = Serial.read();
    //if something has been sent, read what it is and set it equal to 'input'

    if (input == 'r') { //check if 'input' is the character 'r' (red)

      red = 255;
      //red
      green = 0;
      //no green
      blue = 0;
      //no blue

    } else if (input == 'b') { //check if 'input' is the character 'b' (blue)

      red = 0;
      //no red
      green = 0;
      //no green
      blue = 255;
      //blue

    } else if (input == 'o') { //check if 'input' is the character 'o' (orange)

      red = 255;
      //red
      green = 100;
      //some green
      blue = 0;
      //no blue

    } else if (input == 'g') { //check if 'input' is the character 'g' (green)
    
      red = 0;
      //no red
      green = 255;
      //green
      blue = 0;
      //no blue

    } else if (input == 'y') { //check if 'input' is the character 'y' (yellow)

      red = 255;
      //red
      green = 182;
      //medium green
      blue = 0;
      //no blue

    } else if (input == 'p') { //check if 'input' is the character 'p' (purple)

      red = 128;
      //mediium red
      green = 0;
      //no green
      blue = 255;
      //blue

    } else if (input == 'l') { //check if 'input' is the character 'l' (AEMlight)

      red = 60;
      //some red
      green = 112;
      //medium green
      blue = 203;
      //lot of blue

    } else if (input == 'd') { //check if 'input' is the character 'd' (AEMdark)

      red = 16;
      //little red
      green = 84;
      //some green
      blue = 162;
      //medium blue

    } else if (input == '0') { //check if 'input' is the character '0' (off)

      red = 0;
      //no red
      green = 0;
      //no green
      blue = 0;
      //no blue

      Underglow.clear();
      //clear the underglow strips
      Mounted.clear();
      //clear the mounted strips

      Underglow.show();
      //actually set the strips to 'off'
      Mounted.show();
      //actually set the strips to 'off'

    } else if (input == '1') { //chack if 'input' is the character '1'

      speed = 45;
      //set the speed of the animation to 45 milliseconds

    } else if (input == '2') { //check if 'input' is the character '2'

      speed = 10;
      //set the speed of the animation to 10 milliseconds

    } else if (input == 'f') { //check if 'input' is the character 'f'

      gayFade(10, UnderglowLength);
      //runs a function that plays all the hue colors through both the Underglow and Mounted LED strips

    } else if (input == 'w') { //check if 'input' is the character 'w'

      red = 0;
      //no red
      green = 0;
      //no green
      blue = 0;
      //no blue

      Underglow.rainbow(0);
      //make a rainbow along the length of the underglow strip, with a starting hue of 0
      Mounted.rainbow(0);
      //make a rainbow along the length of the mounted strip, with a starting hue of 0

      Underglow.show();
      //t̶a̶s̶t̶e̶ show the rainbow
      Mounted.show();
      //t̶a̶s̶t̶e̶ show the rainbow

    } else if (input == ',') { //check if 'input' is the character ','

      pulse(0, 63, 5);
      //pulse function, pulse from 0 to 63 5 times
      
    }

  }

}

void gayFade(int speed, int length) { //said funtion that plays all the hue colors through both the Underglow and Mounted LED strips

  Underglow.clear();
  //clears the Underglow strip; turns it off
  Mounted.clear();
  //clears the Mounted strip; turns it off

  int* nextColor;
  //declare a pointer variable (points to the address of another variable instead of telling the actual value)

  int red, green, blue;
  //declare COLORS

  for (int i=0; i<(length+763); i++) { //repeat through all the pixels that would show a hue color, the frame offset (bigger than the strip length so the animation doesn't end when the first hue color gets to the end of the strip)

    red = 254; blue = 0; green = 0;
    //resets the first hue color to red at the start of every frame

    for (int j=0; j>-762; j--) { //repeats through all the pixels that will show a hue color for this frame, the hue line

      if (i+j >= 0) { //ignores pixels that are before the strip to save processing power (couldn't find a way to exclude pixels beyond the strip without colors breaking)
        Underglow.setPixelColor(j+i, Underglow.Color(red, green, blue));
        //queue the pixel color to the hue at a pixel index that is the sum of the frame offset and the current index in the hue line
        Mounted.setPixelColor(j+i, Mounted.Color(red, green, blue));
        //queue the pixel color to the hue at a pixel index that is the sum of the frame offset and the current index in the hue line

        nextColor = changeColor(red, green, blue);
        //sets the pointer variable to the result of a function that sets the next hue value so that the resulting array can be read properly

        red = nextColor[0]; green = nextColor[1]; blue = nextColor[2];
        //sets the RGB values to the resulting colors via the pointer variable that points to the result of the color function

      }

    }

    Underglow.setPixelColor(i-762, Underglow.Color(0,0,0));
    //queues the final pixel in the Underglow hue line to off, so that the end of the line turns off instead of leaving a random red pixel
    Mounted.setPixelColor(i-762, Mounted.Color(0,0,0));
    //queues the final pixel in the Mounted hue line to off, so that the end of the line turns off instead of leaving a random red pixel

    Underglow.show();
    //shows the queued Underglow hue colors all at once
    Mounted.show();
    //shows the queued Mounted hue colors all at once

    delay(speed);
    //delay so the animation plays smoothly and doesn't give people a seizure

  }

}

int* changeColor(int red, int green, int blue) { //function that sets the next hue color for the gayFade() function, int* to declare it's result as a pointer variable so gayFade() can read the result properly

  static int Color[3]; //declares the variable that will hold the final RGB values

  if (red == 254 && blue == 0 && green < 254) { //checks if red is at the max value and if green is smaller than its max value (254 because the function increments by 2 so gayFade() doesn't take until the end of time to play)

    Color[0] = red; Color[1] = green+2; Color[2] = blue;
    //sets the resultant variable to the RGB values, incrementing green by 2

  } else if (green == 254 && blue == 0 && red > 0) { //checks if green is at its max f=value and if red is bigger than its min value

    Color[0] = red-2; Color[1] = green; Color[2] = blue;
    //sets the resultant variable to the RGB values, deincrementing red by 2
    
  } else if (green == 254 && red == 0 && blue < 254) { //checks if green is at its max value and if blue is smaller than its max value

    Color[0] = red; Color[1] = green; Color[2] = blue+2;
    //sets the resultant variable to the RGB values, incrementing blue by 2

  } else if (blue == 254 && red == 0 && green > 0) { //checks if blue is at its max value and if green is bigger than its min value

    Color[0] = red; Color[1] = green-2; Color[2] = blue;
    //sets the resultant variable to the RGB values, deincrementing green by 2

  } else if (blue == 254 && green == 0 && red < 254) { //checks if blue is at its max value and if red is smaller than its max value
  
    Color[0] = red+2; Color[1] = green; Color[2] = blue;
    //sets the resultant variable to the RGB values, incrementing red by 2

  } else if (red == 254 && green == 0 && blue > 0) { //checks if red is at its max value and if blue is bigger than its min value

    Color[0] = red; Color[1] = green; Color[2] = blue-2;
    //sets the resultant variable to the RGB values, deincrementing blue by 2

  }

  return Color;
  //returns the final resultant variable to be pointed to by the pointer variable in gayFade()

}

void sawtoothFade(int red, int green, int blue) { //function that plays a sawtooth pattern with the given color

  int subtractValRed, subtractValGreen, subtractValBlue;
  //declare the variables that will hold the value to subtract from each color value to make the sawtooth look good

  int finalRed, finalGreen, finalBlue;
  //declare the variables that will hold the final RGB values, so altering them doesn't mess with the ones declared at the beginning of the code

  for (int i=0; i<UnderglowLength; i++) { //repeat through all of the Underglow strip pixel indexes

    subtractValRed = map(modulo(i + shift, setLength), 0, setLength, 0, red);
    //set the red subtract value to a number that was mapped from a number between 0 and the pattern langth to 0 and the stored red value
    subtractValGreen = map(modulo(i + shift, setLength), 0, setLength, 0, green);
    //set the green subtract value to a number that was mapped from a number between 0 and the pattern length to 0 and the stored green value
    subtractValBlue = map(modulo(i + shift, setLength), 0, setLength, 0, blue);
    //set the blue subtract value to a number that was mapped from a number between 0 and the pattern length to 0 and the stored blue value

    finalRed = round(red - subtractValRed);
    //set the final red value to the stored red value minus the red subtract value
    finalGreen = round(green - subtractValGreen);
    //set the final green value to the stored green value minus the green subtract value
    finalBlue = round(blue - subtractValBlue);
    //set the final blue value to the stored blue value minus the blue subtract value

    Underglow.setPixelColor(i, Underglow.Color(finalRed, finalGreen, finalBlue));
    //queue the pixel color to the final RGB values on the Underglow LED strip
    Mounted.setPixelColor(i, Mounted.Color(finalRed, finalGreen, finalBlue));
    //queue the pixel color to the final RGB values on the Mounted LED strip

  }

  Underglow.show();
  //show the queued pixel colors on the Underglow LED strip all at once
  Mounted.show();
  //show the queued pixel colors on the Mounted LED strip all at once

}

void pulse(int low, int high, int reps) { //pulses the LEDs

  Underglow.fill(Underglow.Color(red, green, blue));
  //fills it one color
  Underglow.show();
  //shows said color

  for (int i=0; i<reps; i++) { //repeats 'reps' times

    for (int b=low; b<=high; b++) { //rising pulse

      Underglow.fill(Underglow.Color(red, green, blue));
      //set the color
      Underglow.setBrightness(b);
      //set the brightness
      Underglow.show();
      //show the color

      delay(speed);
      //delay for effect

    }

    for (int b=high; b>=low; b--) { //falling pulse

      Underglow.fill(Underglow.Color(red, green, blue));
      //set the color
      Underglow.setBrightness(b);
      //set the brightness
      Underglow.show();
      //show the color

      delay(speed);
      //delay for effect
    }

  }

  Underglow.setBrightness(255);
  //reset brightness
  Underglow.show();
  //show brightness

}

//end
//hope you liked the code, make sure to like and subscribe!