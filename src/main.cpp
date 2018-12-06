#include "ofApp.h"
#include "ofAppGlutWindow.h"

int main() {

    //settings for 1024x600 screen as second monitor (if arranged left of main monitor)
    ofGLFWWindowSettings settings;
    settings.windowMode = OF_WINDOW;
    settings.setSize(1920, 1080);
    settings.setPosition({1920.0f,0.0f});
    settings.decorated = false;
    settings.multiMonitorFullScreen = false;
    ofCreateWindow(settings);

    ofRunApp(new ofApp());
}
