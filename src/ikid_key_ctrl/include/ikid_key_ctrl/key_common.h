#ifndef KEY_COMMON_H
#define KEY_COMMON_H
#include <math.h>
#include <string>
#include <vector>
using namespace std;
 
namespace KEY_CTRL {
enum class KB { UP = 1, DOWN, LEFT, RIGHT };
enum class KEY_CLASS { GEER = 1, DIR, START, SPEED };
 
enum class GEER { N = 0, D, R, P };
}  // namespace KEY_CTRL
 
#endif  // IKID_COMMON_H