#pragma once

#include "configuration.h"
#include <OLEDDisplay.h>
#include <functional>
#include <string>

namespace graphics
{

enum VirtualKeyType { VK_CHAR, VK_BACKSPACE, VK_ENTER, VK_SHIFT, VK_ESC, VK_SPACE };

struct VirtualKey {
    char character;
    VirtualKeyType type;
    uint8_t x;
    uint8_t y;
    uint8_t width;
    uint8_t height;
};

class VirtualKeyboard
{
  public:
    VirtualKeyboard();
    ~VirtualKeyboard();

    void draw(OLEDDisplay *display, int16_t offsetX, int16_t offsetY);
    void setInputText(const std::string &text);
    std::string getInputText() const;
    void setHeader(const std::string &header);
    void setCallback(std::function<void(const std::string &)> callback);

    // Navigation methods for encoder input
    void moveCursorUp();
    void moveCursorDown();
    void moveCursorLeft();
    void moveCursorRight();
    void handlePress();
    void handleLongPress();

    // Touch tap support: if (touchX, touchY) falls within a key, move the
    // cursor to that key. Returns true when a key was hit (caller should then
    // call handlePress() to actually type/activate it). False = touch was
    // outside the key grid (e.g. on the input/header area) — ignore it.
    bool selectKeyAt(int16_t touchX, int16_t touchY);

    // Timeout management
    void resetTimeout();
    bool isTimedOut() const;

  private:
    static const uint8_t KEYBOARD_ROWS = 4;
    static const uint8_t KEYBOARD_COLS = 11;
    static const uint8_t KEY_WIDTH = 9;
    static const uint8_t KEY_HEIGHT = 9;        // Compressed to fit 4 rows on 64px displays
    static const uint8_t KEYBOARD_START_Y = 26; // Start just below input box bottom

    VirtualKey keyboard[KEYBOARD_ROWS][KEYBOARD_COLS];

    std::string inputText;
    std::string headerText;
    std::function<void(const std::string &)> onTextEntered;

    uint8_t cursorRow;
    uint8_t cursorCol;

    // Timeout management for auto-exit
    uint32_t lastActivityTime;
    static const uint32_t TIMEOUT_MS = 60000; // 1 minute timeout

    // Pixel layout cache, written at the end of draw() so selectKeyAt() can
    // map a touch coordinate back to a row/col without recomputing geometry.
    int16_t m_lastOffsetY = 0;
    int16_t m_lastKeyTopY = 0;       // pixel Y of the top of the key grid
    int     m_lastCellH = 0;
    int     m_lastColX[KEYBOARD_COLS] = {0};
    int     m_lastColW[KEYBOARD_COLS] = {0};
    bool    m_layoutCached = false;

    void initializeKeyboard();
    void drawKey(OLEDDisplay *display, const VirtualKey &key, bool selected, int16_t x, int16_t y, uint8_t w, uint8_t h,
                 bool isLastCol);
    void drawInputArea(OLEDDisplay *display, int16_t offsetX, int16_t offsetY, int16_t keyboardStartY);

    // Unified cursor movement helper
    void moveCursorDelta(int dRow, int dCol);

    char getCharForKey(const VirtualKey &key, bool isLongPress = false);
    void insertCharacter(char c);
    void deleteCharacter();
    void submitText();
};

} // namespace graphics
