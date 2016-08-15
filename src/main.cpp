#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_WS2801.h>
#include <Task.h>
#include <FunctionTask.h>
#include "ButtonTask.h"

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

#define OFF 0x000000;

/**
 * Prototypes
 **/
void heartbeat(uint32_t deltaTime);
void onSwitchAnimation(uint8_t buttonId, ButtonTask::ButtonState state);

/**
 * Variables
 **/
uint8_t switchPin = 2;
uint8_t dataPin  = 11;    // Yellow wire on Adafruit Pixels
uint8_t clockPin = 13;    // Green wire on Adafruit Pixels
Adafruit_WS2801 strip = Adafruit_WS2801(10, dataPin, clockPin);

TaskManager mTaskManager;
FunctionTask mHeartbeatTask(heartbeat, MsToTaskTime(500));
ButtonTask mButtonTask(onSwitchAnimation, 0, switchPin);

uint32_t mStep = 0;
uint32_t mStepCount = 1;
uint32_t mStepResetCount = 0;
unsigned long mStepDelay = 100;
unsigned long mLastStepMillis = 0;
uint8_t mAnimationIndex = 0;


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
		if (mAnimationIndex >= 4)
		{
			mAnimationIndex = 0;
		}
	}
}

/******************************************************************************
 *
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
		color = 0x990000;
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

	unsigned long now = millis();
	if (now - mLastStepMillis >= mStepDelay)
	{
		/* Move the step index */
		mStep++;
		if (mStep >= mStepCount)
		{
			mStepResetCount++;
			mStep = 0;
		}

		/* Fire the current animation */
		switch(mAnimationIndex)
		{
			default:
			case 0:
				animate88MPH();
			break;

			case 1:
				animateCylon();
			break;

			case 2:
				animateRainbowCycle();
			break;

			case 3:
				animateRainbow();
			break;
		}

		/* Delay the next step */
		mLastStepMillis = now;

	}

	strip.show();
	yield();
}
