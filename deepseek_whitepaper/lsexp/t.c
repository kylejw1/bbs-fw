#include <stdint.h>
static int32_t a,b,c;
int16_t f(int32_t x) { a = x*100 >> 8; b = a + 3; c = b - a; return (int16_t)c; }
