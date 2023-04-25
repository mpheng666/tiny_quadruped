#ifndef BUTTON_
#define BUTTON_

#include <Arduino.h>

namespace button_ns
{
    class Button
    {
    public:
        Button(int8_t pin);
        Button();
        ~Button();
        void init();
        void update();
        int8_t getState();
        bool isPressed();

    private:
        int8_t pin;
        int8_t state;
        int8_t lastReading;
        unsigned long lastDebounceTime = 0;
        unsigned long debounceDelay = 50;
    };
} //button_ns

#endif