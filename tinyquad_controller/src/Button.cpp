#include <Button.hpp>

namespace button_ns
{

    Button::Button(int8_t pin)
    {
        this->pin = pin;
        lastReading = LOW;
        init();
    }

    Button::Button()
    {

    }

    Button::~Button()
    {

    }

    void Button::init()
    {
        pinMode(pin, INPUT_PULLDOWN);
        update();
    }

    void Button::update()
    {
        int8_t newReading = digitalRead(pin);

        if (newReading != lastReading)
        {
            lastDebounceTime = millis();
        }
        if (millis() - lastDebounceTime > debounceDelay)
        {
            state = newReading;
        }
        lastReading = newReading;
    }

    int8_t Button::getState()
    {
        update();
        return state;
    }

    bool Button::isPressed()
    {
        return (getState() == HIGH);
    }
} //button_ns