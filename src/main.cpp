#include "app.h"

void setup() { appSetup(); }
void loop() { appLoop(); }

int main() { // for test env
    setup();
    while (true) {
        loop();
    }
    return 0;
}