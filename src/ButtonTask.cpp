#include <Arduino.h>
#include <Task.h>
#include "ButtonTask.h"


/************************************************
 * Constructor
 ***********************************************/
ButtonTask::ButtonTask(actionCallback function, uint8_t buttonId, uint8_t pin) : Task(MsToTaskTime(3)),
		_buttonId(buttonId),
		_buttonPin(pin),
        _callback(function)
    {
    };

/************************************************
 * Called when the task manager is started in setup().
 ***********************************************/
bool ButtonTask::OnStart()
{
    pinMode(_buttonPin, INPUT_PULLUP);
    _state = RELEASED;
    return true;
}


/***********************************************
 * Fired every 3ms interal.
 **********************************************/
void ButtonTask::OnUpdate(uint32_t deltaTime)
{
    uint16_t deltaTimeMs = TaskTimeToMs(deltaTime);
    ButtonState pinState = (digitalRead(this->_buttonPin) == LOW) ? PRESSED : RELEASED;

    if (pinState != (this->_state & PRESSED))
    {
        if (pinState == PRESSED)
        {
            // just read button down and start timer
            this->_timer = _debouceMs;
            _state = TRACKING;
        }
        else
        {
            if ((_state & TRACKING) == PRESSED) // not tracking
            {
                // triggered released
                _callback(_buttonId, RELEASED);
            }
            _state = RELEASED;
        }
    }
    else
    {
        switch (_state)
        {
        case TRACKING:
            if (deltaTimeMs >= _timer)
            {
                // press debounced
                _state = PRESSED;
                _timer = _repeatDelayMs;
                _callback(_buttonId, PRESSED);
            }
            else
            {
                _timer -= deltaTimeMs;
            }
            break;

        case PRESSED:
            if (deltaTimeMs >= _timer)
            {
                // auto repeat started
                _state = AUTO_REPEAT;
                _timer = _repeatRateMs;
                _callback(_buttonId, AUTO_REPEAT);
            }
            else
            {
                _timer -= deltaTimeMs;
            }
            break;

        case AUTO_REPEAT:
            if (deltaTimeMs >= _timer)
            {
                // auto repeat triggered again
                _timer += _repeatRateMs;
                _callback(_buttonId, AUTO_REPEAT);
            }
            else
            {
                _timer -= deltaTimeMs;
            }
            break;
        }
    }
}
