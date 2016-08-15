#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_WS2801.h>
#include <Task.h>
#include <FunctionTask.h>
#include "ButtonTask.h"

#define arrayLen(x)  (sizeof(x) / sizeof(x[0]))

#define B0 0
#define B1 1
#define B2 2
#define R0 3
#define R1 4
#define R2 5
#define L0 6
#define L1 7
#define L2 8
#define STATUS 9
#define LAST L2
#define ANALOG_MAX 1024
#define SPEED_SCALE_RANGE 3.0
#define SPEED_SCALE_MIN 0.04

#define OFF 0x000000;

/**
 * Prototypes.. due to the order of functions in this file, I only needed these 2 for Task.h
 **/
void heartbeat(uint32_t deltaTime);
void onSwitchAnimation(uint8_t buttonId, ButtonTask::ButtonState state);

/* The animation cycle switch input pin */
uint8_t switchPin = PD2;

/* The SPI data pin, yellow wire on Pixel String */
uint8_t dataPin  = PD3;

/* The SPI clock pin, green wire on pixel string */
uint8_t clockPin = PD4;

/* The analog speed adjustment input pin */
uint8_t analogInputPin = A0;

/* The strip of ADAFruit pixels.  My FluxCapacitor has 9 main pixels, and 1 status pixel at the end */
Adafruit_WS2801 strip = Adafruit_WS2801(10, dataPin, clockPin);

/* Task.h manager and callback functions.  One to to update the status pixel, one to track button inputs */
TaskManager mTaskManager;
FunctionTask mHeartbeatTask(heartbeat, MsToTaskTime(500));
ButtonTask mButtonTask(onSwitchAnimation, 0, switchPin);

/* The current animation step */
uint32_t mStep = 0;

/* The number of steps to perform, before resetting the step count back to zero */
uint32_t mStepCount = 1;

/* Track each time the step is reset back to zero */
uint32_t mStepResetCount = 0;

/* The number of milliseconds between each step executing */
unsigned long mStepDelay = 100;

/* The time of the last step, so we know if it's time to fire the next step */
unsigned long mLastStepMillis = 0;

/* A list of animation functions that can be cycled through */
void (* mAnimationFunctions [5])();

/* The current index into the mAnimationFunctions array */
uint8_t mAnimationIndex = 0;

/* The calculated step speed scale, derived from the analog input pin */
float mSpeedScale = 1.0; /* Cycles between SPEED_SCALE_MIN and SPEED_SCALE_MAX */

/* Used by the optional heartbeat pixel (#10) and just steps through a few colors off/on */
uint32_t mHeartBeatColors[] = {0x020000, 0x000200, 0x000002};
uint8_t mHeartBeatIndex = 0;

/******************************************************************************
 *
 *****************************************************************************/
void onSwitchAnimation(uint8_t buttonId, ButtonTask::ButtonState state)
{
	if (ButtonTask::ButtonState::PRESSED == state)
	{
		mStepResetCount = 0;
		mStep = 0;
		mStepCount = 1;
		mAnimationIndex++;
		if (mAnimationIndex >= arrayLen(mAnimationFunctions))
		{
			mAnimationIndex = 0;
		}
	}
}

/******************************************************************************
 * Just cycle the 10th pixel around the array of blink colors
 ******************************************************************************/
void heartbeat(uint32_t deltaTime)
{

	uint32_t color = strip.getPixelColor(STATUS);

	if (color > 0)
	{
		color = 0x00;
	}
	else
	{
		mHeartBeatIndex++;
		if (mHeartBeatIndex >= arrayLen(mHeartBeatColors))
		{
			mHeartBeatIndex = 0;
		}

		color = mHeartBeatColors[mHeartBeatIndex];
	}

	strip.setPixelColor(STATUS, color);
}


/**********************************************************************************
 * Create a 24 bit color value from R,G,B
 *********************************************************************************/
uint32_t toColor(byte r, byte g, byte b)
{
  uint32_t c;
  c = r;
  c <<= 8;
  c |= g;
  c <<= 8;
  c |= b;
  return c;
}

/********************************************************************************
* Input a value 0 to 255 to get a color value.
* The colors are a transition r - g -b - back to r
*********************************************************************************/
uint32_t wheelColor(byte WheelPos)
{
  if (WheelPos < 85) {
   return toColor(WheelPos * 3, 255 - WheelPos * 3, 0);
  } else if (WheelPos < 170) {
   WheelPos -= 85;
   return toColor(255 - WheelPos * 3, 0, WheelPos * 3);
  } else {
   WheelPos -= 170;
   return toColor(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}


/******************************************************************************
 *
 *****************************************************************************/
void allOff()
{
	for (int index = 0; index <= LAST; index++)
	{
		strip.setPixelColor(index, 0);
	}
}

/*****************************************************************************
 *
 ****************************************************************************/
void animateOff()
{
	mStepDelay = 250;
	mStepCount = 2;
	allOff();
}

/******************************************************************************
 *
 *****************************************************************************/
void animate88MPH()
{

	/* Every 10 resets, change the speed */
	mStepCount = mStepResetCount / 10 % 2 == 0 ? 10 : 14;
	mStepDelay = mStepResetCount / 10 % 2 == 0 ? 40 : 70;

	uint32_t on = 0x00aaaa;
	uint32_t trail = 0x001111;
	uint32_t trail2 = 0x000f0f;

	uint32_t r0, r1, r2 = OFF;

	switch(mStep)
	{
		case 0:
			r2 = on;
			r1 = OFF;
			r0 = OFF;
		break;

		case 1:
			r2 = trail;
			r1 = on;
			r0 = OFF;
		break;

		case 2:
			r2 = trail2;
			r1 = trail;
			r0 = on;
		break;

		case 3:
			r2 = OFF;
			r1 = trail2;
			r0 = trail;
		break;

		case 4:
			r2 = OFF;
			r1 = OFF;
			r0 = trail2;
		break;

		default:
			r0 = r1 = r2 = OFF;
		break;
	}


	strip.setPixelColor(L2, r2);
	strip.setPixelColor(R2, r2);
	strip.setPixelColor(B2, r2);

	strip.setPixelColor(L1, r1);
	strip.setPixelColor(R1, r1);
	strip.setPixelColor(B1, r1);

	strip.setPixelColor(L0, r0);
	strip.setPixelColor(R0, r0);
	strip.setPixelColor(B0, r0);

}

/******************************************************************************
 *
 *****************************************************************************/
void animateCylon()
{
	mStepCount = 10;
	mStepDelay = 150;

	allOff();

	uint16_t pixel = L2;

	switch(mStep)
	{
		case 0:
			pixel = L2;
		break;

		case 1:
		case 9:
			pixel = L1;
		break;

		case 2:
		case 8:
			pixel = L0;
		break;

		case 3:
		case 7:
			pixel = R0;
		break;

		case 4:
		case 6:
			pixel = R1;
		break;

		case 5:
			pixel = R2;
		break;

	}

	strip.setPixelColor(pixel, 0xff0000);
}


/**************************************************************************
 * Pulled from the Adafruit site.
 **************************************************************************/
void animateRainbow()
{
	mStepCount = 256;
	mStepDelay = 25;

	uint16_t i;
    for (i=0; i < strip.numPixels()-1; i++)
	{
      strip.setPixelColor(i, wheelColor( (i + mStep) % 255));
    }

}

/****************************************************************************
 * Pulled from the Adafruit site.
 ***************************************************************************/
void animateRainbowCycle()
{
	mStepCount = 256;
	mStepDelay = 15;

	uint16_t i;

    for (i=0; i < strip.numPixels()-1; i++)
	{
      // tricky math! we use each pixel as a fraction of the full 96-color wheel
      // (thats the i / strip.numPixels() part)
      // Then add in j which makes the colors go around per pixel
      // the % 96 is to make the wheel cycle around
      strip.setPixelColor(i, wheelColor( ((i * 256 / (strip.numPixels()-1)) + mStep) % 256) );
    }

}


/******************************************************************************
 *
 *****************************************************************************/
void setup()
{
	Serial.begin(9600);

	mAnimationFunctions[0] = animate88MPH;
	mAnimationFunctions[1] = animateCylon;
	mAnimationFunctions[2] = animateRainbowCycle;
	mAnimationFunctions[3] = animateRainbow;
	mAnimationFunctions[4] = animateOff;

	mTaskManager.StartTask(&mHeartbeatTask);
	mTaskManager.StartTask(&mButtonTask);

	pinMode(switchPin, INPUT_PULLUP);

	strip.begin();
	allOff();
	strip.show();
}

/******************************************************************************
 *
 *****************************************************************************/
void loop()
{
	mTaskManager.Loop();

	/* Calculate the analog speed adjustment */
	int analogIn = analogRead(analogInputPin);	mSpeedScale = ((float)analogIn) * SPEED_SCALE_RANGE / ANALOG_MAX;
	mSpeedScale = SPEED_SCALE_RANGE - mSpeedScale; /* I wired the left and right pins of my pot backwards.. so .. reverse the final value */
	mSpeedScale += SPEED_SCALE_MIN;

	unsigned long now = millis();
	if (now - mLastStepMillis >= (mStepDelay * mSpeedScale))
	{
		/* Move the step index */
		mStep++;
		if (mStep >= mStepCount)
		{
			mStepResetCount++;
			mStep = 0;
		}

		/* Fire the current animation */
		(*mAnimationFunctions[mAnimationIndex])();

		/* Delay the next step */
		mLastStepMillis = now;

	}

	strip.show();
	yield();
}
