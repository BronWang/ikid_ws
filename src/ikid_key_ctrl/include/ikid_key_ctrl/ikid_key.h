#ifndef IKID_KEY_H
#define IKID_KEY_H
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
 
class Keyboard_ctrl {
  struct termios initial_settings, new_settings;
  int peek_character = -1;
 
 public:
  Keyboard_ctrl() { init_keyboard(); };
  ~Keyboard_ctrl() { close_keyboard(); };
 
  int get_keyboard_press_key() {
    kbhit();
    return readch();
    // printf("%02x \n", readch());
  };
 
 private:
  void init_keyboard() {
    tcgetattr(0, &initial_settings);
    new_settings = initial_settings;
    new_settings.c_lflag &= ~(ICANON | ECHO);
    new_settings.c_cc[VEOL] = 1;
    new_settings.c_cc[VEOF] = 2;
    tcsetattr(0, TCSANOW, &new_settings);
  }
 
  void close_keyboard() { tcsetattr(0, TCSANOW, &initial_settings); }
 
  int kbhit() {
    unsigned char ch;
    int nread;
 
    if (peek_character != -1) return 1;
    new_settings.c_cc[VMIN] = 0;
    tcsetattr(0, TCSANOW, &new_settings);
    nread = read(0, &ch, 1);
    new_settings.c_cc[VMIN] = 1;
    tcsetattr(0, TCSANOW, &new_settings);
    if (nread == 1) {
      peek_character = ch;
      return 1;
    }
    return 0;
  }
 
  int readch() {
    char ch;
    int nread;
    if (peek_character != -1) {
      ch = peek_character;
      peek_character = -1;
      return ch;
    }
    nread = read(0, &ch, 1);
    return ch;
  }
};
 
#endif  // KEY_H
