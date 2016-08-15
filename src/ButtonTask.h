#include <Task.h>

/************************************************
 * A variation of ButtonTask.h from the examples.
 ************************************************/
class ButtonTask : public Task
{
	public:

		enum ButtonState
		{
		    RELEASED =   0b00000000,
		    PRESSED =    0b00000001,
		    AUTO_REPEAT = 0b00000011,
		    TRACKING =   0b10000001
		};

	    typedef void(*actionCallback)(uint8_t buttonId, ButtonState state);

	    ButtonTask(actionCallback function, uint8_t buttonId, uint8_t pin);

	private:
	    static const uint16_t _debouceMs = 50; // (30-100) are good values
	    static const uint16_t _repeatDelayMs = 600; // (400 - 1200) are reasonable values
	    static const uint16_t _repeatRateMs = 50; // (40-1000) are reasonable
		const uint8_t _buttonId;
	    const uint8_t _buttonPin;
	    const actionCallback _callback;
	    uint16_t _timer;
	    ButtonState _state;

	    bool OnStart();
	    void OnUpdate(uint32_t deltaTime);

};
