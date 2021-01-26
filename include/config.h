#ifndef _CONFIG_H_
#define _CONFIG_H_

// -------------- Hardware interfaces ----------------
const int LED_BUILTIN = 2;

//-------------------------------------------------//
// Function to return the sign of a value
//-------------------------------------------------//
template<typename T>
static constexpr int sgn(const T val) {   
 if (val < 0) return -1;
 if (val == 0) return 0;
 return 1;
}

// -------------------------------------------------------//
// OLED configuration                                     //
// -------------------------------------------------------//
#define USE_OLED true

#endif // _CONFIG_H_