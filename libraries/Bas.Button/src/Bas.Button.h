/*
  Bas.Button.h - Library for a debounced button that can call calllbacks on rising or falling action.
  Created by Bas Paap, August 2023.
  Released into the public domain.
*/

#ifndef _BUTTON_h
#define _BUTTON_h

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif
#include <CallbackCaller.h>
namespace Bas
{
	/// <summary>
	/// Encapsulate a button connected to a pull-up resistor. If the button is pressed or released (and any bouncing is filtered out), a callback is called.
	/// </summary>
	class Button
	{
	public:
		enum class LogLevel
		{
			none = 0,
			normal
		};

	private:
		int pin;
		int lastDebouncedButtonState = HIGH;
		unsigned long lastDebounceTime;
		unsigned long debounceDelay;
		CallbackCaller risingCallbackCaller;
		CallbackCaller fallingCallbackCaller;
		int debouncedState;
		LogLevel logLevel;

	public:
		/// <summary>
		/// Constructs the Button object.
		/// </summary>
		/// <param name="pin">The pin the button is connected to.</param>
		/// <param name="debounceDelay">The delay (in millis) to use for filtering any bouncing.</param>
		Button(int pin, unsigned long debounceDelay, LogLevel logLevel = LogLevel::none);

		/// <summary>
		/// Tell the button which callback to call when the signal falls (in other words, when the button is pressed).
		/// </summary>
		/// <param name="fallingCallback">The callback to call when the button is pressed.</param>
		// template <typename Function>
		// void begin(Function fallingCallback)
		// {
		// 	this->begin(fallingCallback, nullptr);
		// }

		/// <summary>
		/// Tell the button which callback to call when the signal falls (when the button is pressed) or rises (when the button is released).
		/// </summary>
		/// <param name="fallingCallback">The callback to call when the button is pressed.</param>
		/// <param name="risingCallback">The callback to call when the button is released.</param>
		template <typename Function>
		void begin(Function fallingCallback, Function risingCallback = []{})
		{
			if (this->logLevel == LogLevel::normal)
			{
				Serial.print("Initializing button on pin ");
				Serial.println(this->pin);
			}

			this->fallingCallbackCaller.begin(fallingCallback);
			this->risingCallbackCaller.begin(risingCallback);
			pinMode(this->pin, INPUT_PULLUP);
			lastDebouncedButtonState = digitalRead(this->pin);
		}

		/// <summary>
		/// Updates the Button state. This method should be called once per loop.
		/// </summary>
		void update();

		/// <summary>
		/// Returns a boolean that specifies if the button is pressed.
		/// </summary>
		/// <returns>A boolean that specifies if the button is pressed.</returns>
		bool isPressed();
	};
}

#endif
