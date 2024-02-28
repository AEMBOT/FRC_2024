//FRC 2024 LED controller, team 6443 AEMBOT, written by c2bVids

/*
~Version History~
 V1 - Added main functions
 V2 - Added more comments, version history, and off button support
 V3 - Added 'pulse' function (I got bored)
 V4 - Added comments to off button support (don't know why I didn't do that already), and optimized 'gayFade' function to use less memory and power
 V5 - Replaced the old 'gayFade' function and the 'changeColor' function with a better version, removed the 'pulse' function
 V6 - Added more features to 'gayFade' function, capitalized the 'r' in the previous version's update
 V7 - Added a function to handle the start of the Underglow strip not being in the middle of the front or back, while still making patterns look cool, updated gayFade's method of exiting function
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
#define UnderglowLength 124
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

const int UnderglowHalf = 35;

const int setLength = 10;

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

  getColor();
  //gets a viable color, speed, or function sent via the Serial port

  if (red != 0 || green != 0 || blue != 0) { //check if the current color isn't 'off'

    sawtoothFade(red, green, blue);
    //makes a sawtooth pattern on the LED strips

    shift = modulo(shift-1, setLength);
    //increments the shift variable, moving the animation, but doesn't let the variable go over the pattern length to save memory

    delay(speed);
    //delay so the shift between frames isn't instant
  }

}

int turnOn(int UnderglowSpeed, int MountedSpeed, int red, int green, int blue) { //function to turn on the LED strips, but *cool*

  for (int i=0; i<UnderglowLength; i++) { //loops through all of the pixel indexes on the Underglow LED strips

    setUnderglowColor(i, red, green, blue, false);
    //queues the given color to a pixel at a given index on the Underglow LED strip

    Underglow.show();
    //show any queued LED colors

    delay(UnderglowSpeed);
    //delay so the strip doesn't just light up all at once

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

  }

  return 0;
  //end the function, gets mad if i don't put this

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

      red = 0;
      green = 0;
      blue = 0;

      gayFade();
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

    }

  }

}

void sawtoothFade(int red, int green, int blue) { //function that plays a sawtooth pattern with the given color

  int subtractValRed, subtractValGreen, subtractValBlue;
  //declare the variables that will hold the value to subtract from each color value to make the sawtooth look good

  int finalRed, finalGreen, finalBlue;
  //declare the variables that will hold the final RGB values, so altering them doesn't mess with the ones declared at the beginning of the code

  for (int i=0; i<UnderglowLength/2; i++) { //repeat through all of the Underglow strip pixel indexes

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

    setUnderglowColor(i, finalRed, finalGreen, finalBlue, false);
    //queue the pixel color to the final RGB values on the Underglow LED strip
    Mounted.setPixelColor(i, Mounted.Color(finalRed, finalGreen, finalBlue));
    //queue the pixel color to the final RGB values on the Mounted LED strip

  }

  Underglow.setBrightness(127);
  //set underglow brightness to half brightness
  Mounted.setBrightness(127);
  //set mounted brightness to half brightness

  Underglow.show();
  //show the queued pixel colors on the Underglow LED strip all at once
  Mounted.show();
  //show the queued pixel colors on the Mounted LED strip all at once

}

void gayFade() { //function to play a parade of hue colors until a new input is sent through Serial port

  int fadeShift = 0;
  //pattern offset value

  int lastColor[3];
  //declare an array to hold the last stored RGB values

  unsigned int hue;
  //declaring hue variable that holds the hue value per pixel, unsigned so that it doesn't try to enter -4.3 billion as a viable hue (I love 2's complement)

  while (true) { //repeats forever

    for (int i=0; i<UnderglowLength/2; i++) { //repeats through all the pixels in the strips

      hue = (i+fadeShift)*182;
      //sets the hue variable for the current pixel, which is the index of the current pixel added to the offset, multiplied by 65535 (max hue value for ColorHSV()) divided by 180 (hue colors per repeat)

      setUnderglowColor(i, 65535-hue, 255, 255, true);
      //queues the pixel at the index using an HSV to RGB converter (built-in to the neopixel library) (using 65535-hue to invert hue)
      Mounted.setPixelColor(i, Mounted.ColorHSV(65535-hue, 255, 255));
      //queues the pixel at the index using an HSV to RGB converter (built-in to the neopixel library) (using 65535-hue to invert hue)

    }

    Underglow.setBrightness(127);
    //sets the brightness of the underglow strips to half brightness
    Mounted.setBrightness(127);
    //sets the brightness of the mounted strips to half brightness

    Underglow.show();
    //shows the queued colors on the underglow strips
    Mounted.show();
    //shows the queued colors on the mounted strips

    fadeShift = modulo((fadeShift-1), 360);
    //increments the shift variable, modding it to 180 (the number of hue colors displayed)

    delay(speed);
    //delay for effect

    lastColor[0] = red; lastColor[1] = green; lastColor[2] = blue;
    //set the array to the last stored RGB color values

    getColor();
    //run the getColor function to check if any other inputs were sent

    if (red != lastColor[0] || green != lastColor[1] || blue != lastColor[2]) { //check if the color has changed at all

      break;
      //break out of the function if it has
      
    }

  }
  
}

void setUnderglowColor(int index, int red, int green, int blue, bool HSV) { //function to deal with the start of the Underglow strip not being centered in the front or back of the robot

  if (HSV) { //check if using HSV or RGB

    Underglow.setPixelColor(index+UnderglowHalf+1, Underglow.ColorHSV(red, green, blue));
    //set the regular section of the strip, just using the index and an offset

    if (index > UnderglowHalf) { //check if the index is at the very end of the strip

      Underglow.setPixelColor(UnderglowLength-(index-UnderglowHalf), Underglow.ColorHSV(red, green, blue));
      //set the pixel color by subtracting UnderglowHalf from index, then subtracting that from the full strip length

    } else if (index <= UnderglowHalf) { //check if the index is less than UnderglowHalf

      Underglow.setPixelColor(UnderglowHalf-index, Underglow.ColorHSV(red, green, blue));
      //set the pixel color by subtracting the index from UnderglowHalf

    }

  } else { //not using HSV

    Underglow.setPixelColor(index+UnderglowHalf+1, Underglow.Color(red, green, blue));
    //set the regular section of the strip, just using the index and an offset

    if (index > UnderglowHalf) { //check if the index is at the very end of the strip

      Underglow.setPixelColor(UnderglowLength-(index-UnderglowHalf), Underglow.Color(red, green, blue));
      //set the pixel color by subtracting UnderglowHalf from index, then subtracting that from the full strip length

    } else if (index <= UnderglowHalf) { //check if the index is less than UnderglowHalf

      Underglow.setPixelColor(UnderglowHalf-index, Underglow.Color(red, green, blue));
      //set the pixel color by subtracting the index from UnderglowHalf

    }

  }

}

//end
//hope you liked the code, make sure to like and subscribe!