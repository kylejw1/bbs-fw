#include <stdint.h>
static int16_t a,b,c;
int16_t f(int16_t x) { a = x*100 >> 8; b = a + 3; c = b - a; return c; }
