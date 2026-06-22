/*
  Bas.Button.h - Library for a debounced button that can call calllbacks on rising or falling action.
  Created by Bas Paap, August 2023.
  Released into the public domain.
*/

#include "Bas.Button.h"

namespace Bas
{
	Button::Button(int pin, unsigned long debounceDelay, LogLevel logLevel) : pin{ pin }, debounceDelay{ debounceDelay }, logLevel { logLevel }
	{
	}

	void Button::update()
	{
		int currentButtonState = digitalRead(this->pin);
		unsigned long now = millis();

		if (currentButtonState != this->lastDebouncedButtonState)
		{
			if (now - this->lastDebounceTime > this->debounceDelay)
			{
				// A debounced button change has occurred!
				this->lastDebounceTime = now;
				this->lastDebouncedButtonState = currentButtonState;
				this->debouncedState = currentButtonState;

				// Call the appropriate callback function
				if (currentButtonState == HIGH)
				{
					if (this->logLevel == LogLevel::normal)
					{
						Serial.println("Button debounced on HIGH.");
					}

					this->risingCallbackCaller.call();
				}

				if (currentButtonState == LOW)
				{
					if (this->logLevel == LogLevel::normal)
					{
						Serial.println("Button debounced on LOW.");
					}

					this->fallingCallbackCaller.call();
				}
			}
		}
	}

	bool Button::isPressed()
	{
		return this->lastDebouncedButtonState == LOW;
	}
}
