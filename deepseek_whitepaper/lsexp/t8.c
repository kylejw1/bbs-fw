#include <stdint.h>
int16_t f(int32_t x,int32_t y,int32_t z) {
 int32_t a=x*7,b=y*7,c=z*7,d=a+b,e=c-d,g=e>>3,h=g+a,i=h-b,j=i+c,k=j-d,l=k+e,m=l-g;
 return (int16_t)(m+h+i+j+k);
}
