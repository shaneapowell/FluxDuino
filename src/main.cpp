#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_WS2801.h>
#include <Task.h>
#include <FunctionTask.h>
#include "ButtonTask.h"

/*******************************/
/* User Controllable Values    */

/* Enable or Disable the use of hte analog pin for speed control
 * (true / false) */
#define ENABLE_ANALOG_SPEED_CONTROL 		true

/* If you wired your left and right pins on your 3 pin POT backwards.. like I did, set this to true to reverse the
 * effect of the POT.  I preferred it when clockwise made the animations faster.
 * (true / false) */
#define REVERSE_ANALOG_SPEED_CONTROL		true

/* The animation cycle switch input pin.  This is the digital input pin, that will be set to PULLUP,
 * and triggered when pulled to ground */
#define ANIMATION_CHANGE_SWITCH_INPUT_PIN 	PD2

/* The SPI data pin, yellow wire on Pixel String */
#define SPI_DATA_PIN						PD3

/* The SPI clock pin, green wire on pixel string */
#define SPI_CLOCK_PIN						PD4

/* The analog speed adjustment input pin.  This is the POT pin that is read and compoared to vREF (0-1024) to determine
 * the speed of the animations.  the center pin should be connected to vRef, and the other pin connected to ground. */
#define ANALOG_SPEED_CONTROL_INPUT_PIN		A0

/* end user controllable values */
/*******************************/


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

/**
 * Constants.  Used instead of directly using the #define values to help ensure type safety
 **/
static const uint16_t armB[] = {B0, B1, B2};
static const uint16_t armL[] = {L0, L1, L2};
static const uint16_t armR[] = {R0, R1, R2};
static const uint16_t ring0[] = {R0, L0, R0};
static const uint16_t ring1[] = {R1, L1, R1};
static const uint16_t ring2[] = {R2, L2, R2};
static const uint8_t switchPin = ANIMATION_CHANGE_SWITCH_INPUT_PIN;
static const uint8_t dataPin  = SPI_DATA_PIN;
static const uint8_t clockPin = SPI_CLOCK_PIN;
static const uint8_t analogInputPin = ANALOG_SPEED_CONTROL_INPUT_PIN;


/* The strip of ADAFruit pixels.  My FluxCapacitor has 9 main pixels, and 1 status pixel at the end */
Adafruit_WS2801 mLedStrip = Adafruit_WS2801(10, dataPin, clockPin);

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
void (* mAnimationFunctions [9])();

/* The current index into the mAnimationFunctions array */
uint8_t mAnimationIndex = 1; /* Start at 1, since 0 is off */

/* The calculated step speed scale, derived from the analog input pin */
float mSpeedScale = 1.0; /* Cycles between SPEED_SCALE_MIN and SPEED_SCALE_MAX */

/* Used by the optional heartbeat pixel (#10) and just steps through a few colors off/on */
uint32_t mHeartBeatColors[] = {0x000200, 0x001200, 0x002200, 0x003200};
uint8_t mHeartBeatIndex = 0;

/******************************************************************************
 * Respond  to the switch animate switch input
 *****************************************************************************/
void onSwitchAnimation(uint8_t buttonId, ButtonTask::ButtonState state)
{
	if (ButtonTask::ButtonState::PRESSED == state)
	{
		mLastStepMillis = 0;
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
 * Just cycle the 10th pixel in the strip around the array of blink colors
 ******************************************************************************/
void heartbeat(uint32_t deltaTime)
{

	uint32_t color = mLedStrip.getPixelColor(STATUS);

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

	mLedStrip.setPixelColor(STATUS, color);
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
uint32_t wheelColor(byte wheelPos)
{
	if (wheelPos < 85)
	{
		return toColor(wheelPos * 3, 255 - wheelPos * 3, 0);
	}
	else if (wheelPos < 170)
	{
		wheelPos -= 85;
		return toColor(255 - wheelPos * 3, 0, wheelPos * 3);
	}
	else
	{
		wheelPos -= 170;
		return toColor(0, wheelPos * 3, 255 - wheelPos * 3);
	}
}


/******************************************************************************
 * Turn off all pixels
 *****************************************************************************/
void allOff()
{
	for (int index = 0; index <= LAST; index++)
	{
		mLedStrip.setPixelColor(index, 0);
	}
}

/*****************************************************************************
 * animate off. Yes.. animate off.  Just  a quick and lazy "off" option.
 ****************************************************************************/
void animateOff()
{
	mStepDelay = 250;
	mStepCount = 2;
	allOff();
}

/******************************************************************************
 * The whole point of this.. do go back in time!!
 *****************************************************************************/
void animate88MPH()
{

	/* Every 10 resets, change the speed */
	mStepCount = mStepResetCount / 10 % 2 == 0 ? 10 : 14;
	mStepDelay = mStepResetCount / 10 % 2 == 0 ? 40 : 70;

	uint32_t on = 0x00aaaa;
	uint32_t trail = 0x001111;
	uint32_t trail2 = 0x00020f;

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


	mLedStrip.setPixelColor(L2, r2);
	mLedStrip.setPixelColor(R2, r2);
	mLedStrip.setPixelColor(B2, r2);

	mLedStrip.setPixelColor(L1, r1);
	mLedStrip.setPixelColor(R1, r1);
	mLedStrip.setPixelColor(B1, r1);

	mLedStrip.setPixelColor(L0, r0);
	mLedStrip.setPixelColor(R0, r0);
	mLedStrip.setPixelColor(B0, r0);

}

/******************************************************************************
 * Make the top to arms animate like the eye of a Cylon.
 *****************************************************************************/
void animateCylon()
{
	static const uint16_t pixels[] = {L2, L1, L0, R0, R1, R2, R1, R0, L0, L1};
	mStepCount = arrayLen(pixels);
	mStepDelay = 150;

	allOff();
	mLedStrip.setPixelColor(pixels[mStep], 0xff0000);
}

/******************************************************************************
 * Yeah.. ok. not a cylon, but all 3 arms.. so a Trylon!!
 *****************************************************************************/
void animateTrylon()
{
	static const uint16_t pixels[] = {L0, L1, L2, L1, L0, R0, R1, R2, R1, R0, B0, B1, B2, B1, B0};
	mStepCount = arrayLen(pixels);
	mStepDelay = 150;

	allOff();
	mLedStrip.setPixelColor(pixels[mStep], 0x0000ff);
}



/**************************************************************************
 * Pulled from the Adafruit site.
 **************************************************************************/
void animateRainbow()
{
	mStepCount = 256;
	mStepDelay = 25;

	uint16_t i;
    for (i=0; i < mLedStrip.numPixels()-1; i++)
	{
      mLedStrip.setPixelColor(i, wheelColor( (i + mStep) % 255));
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

    for (i=0; i < mLedStrip.numPixels()-1; i++)
	{
      // tricky math! we use each pixel as a fraction of the full 96-color wheel
      // (thats the i / strip.numPixels() part)
      // Then add in j which makes the colors go around per pixel
      // the % 96 is to make the wheel cycle around
      mLedStrip.setPixelColor(i, wheelColor( ((i * 256 / (mLedStrip.numPixels()-1)) + mStep) % 256) );
    }

}

/****************************************************************************
 * Variation of the above rainbow cycle, keeps the 3 arms in sync
 ***************************************************************************/
void animateRainbowFlux()
{
	mStepCount = 256;
	mStepDelay = 8;

	uint32_t color;
	uint16_t index;
	uint16_t pixelIndex;

    for (index = 0; index < arrayLen(armB); index++)
	{
		pixelIndex = arrayLen(armB) - index - 1;
		color = wheelColor( ((index * 256 / 8) + mStep) % 256);
		mLedStrip.setPixelColor(armB[pixelIndex], color );
		mLedStrip.setPixelColor(armL[pixelIndex], color );
		mLedStrip.setPixelColor(armR[pixelIndex], color );
    }

}

/***************************************************************************
 * Another variation on the rainbow, this one bounces the rings in and out
 **************************************************************************/
void animateRainbowBounce()
{
	static const uint16_t rings[] = {0, 1, 2, 1};
	mStepCount = arrayLen(rings);
	mStepDelay = 100;

	uint32_t color= wheelColor( ((mStepResetCount * 256 / 20) + mStepResetCount) % 256);

	allOff();
	mLedStrip.setPixelColor(armL[rings[mStep]], color);
	mLedStrip.setPixelColor(armR[rings[mStep]], color);
	mLedStrip.setPixelColor(armB[rings[mStep]], color);

}

/*****************************************************************************
 *
 ****************************************************************************/
void animateSpirol()
{
	static const uint16_t pixels[] = {B0, L0, R0, B1, L1, R1, B2, L2, R2};
	mStepCount = arrayLen(pixels);
	mStepDelay = 150;
	allOff();
	uint32_t color = wheelColor( ((mStepResetCount * 256 / 20) + mStepResetCount) % 256);
	mLedStrip.setPixelColor(pixels[mStep], color);
}


/******************************************************************************
 *
 *****************************************************************************/
void setup()
{
	Serial.begin(9600);

	/*  Don't forget to update the size of the function pointer array when adding a new one here */
	mAnimationFunctions[0] = animateOff;
	mAnimationFunctions[1] = animate88MPH;
	mAnimationFunctions[2] = animateCylon;
	mAnimationFunctions[3] = animateTrylon;
	mAnimationFunctions[4] = animateRainbowCycle;
	mAnimationFunctions[5] = animateRainbow;
	mAnimationFunctions[6] = animateRainbowFlux;
	mAnimationFunctions[7] = animateRainbowBounce;
	mAnimationFunctions[8] = animateSpirol;

	mTaskManager.StartTask(&mHeartbeatTask);
	mTaskManager.StartTask(&mButtonTask);

	pinMode(switchPin, INPUT_PULLUP);

	mLedStrip.begin();
	allOff();
	mLedStrip.show();
}

/******************************************************************************
 *
 *****************************************************************************/
void loop()
{
	mTaskManager.Loop();

	/* Calculate the analog speed adjustment */
	if (ENABLE_ANALOG_SPEED_CONTROL)
	{
		int analogIn = analogRead(analogInputPin);	mSpeedScale = ((float)analogIn) * SPEED_SCALE_RANGE / ANALOG_MAX;
		if (REVERSE_ANALOG_SPEED_CONTROL)
		{
			mSpeedScale = SPEED_SCALE_RANGE - mSpeedScale;
		}
		mSpeedScale += SPEED_SCALE_MIN;
	}

	/* do an animation step */
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

		mLedStrip.show();

	}

	yield();
}
