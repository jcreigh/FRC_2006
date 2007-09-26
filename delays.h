#ifndef __DELAYS_H
#define __DELAYS_H

/* PIC 17Cxxx and 18Cxxx cycle-count delay routines.
 *
 *   Functions:
 *		 Delay1TCY()
 *               Delay10TCY()  // 17Cxx only
 *               Delay10TCYx()
 *               Delay100TCYx()
 *               Delay1KTCYx()
 *               Delay10KTCYx()
 */

/* Delay of exactly 1 Tcy */
#define Delay1TCY() Nop() 

#if __18CXX
#define PARAM_SCLASS auto
#else
#define PARAM_SCLASS static
#endif

/* Delay of exactly 10 Tcy */
#if    __18CXX
#define Delay10TCY() Delay10TCYx(1)
#else /* 17CXX */
far void Delay10TCY(void);
#endif

/* Delay10TCYx
 * Delay multiples of 10 Tcy
 * Passing 0 (zero) results in a delay of 2560 cycles.
 * The 18Cxxx version of this function supports the full range [0,255]
 * The 17Cxxx version supports [2,255] and 0.
 */
#if    __18CXX
void Delay10TCYx(PARAM_SCLASS unsigned char);
#else /* 17CXX */
far void Delay10TCYx(PARAM_SCLASS unsigned char);
#endif

/* Delay100TCYx
 * Delay multiples of 100 Tcy
 * Passing 0 (zero) results in a delay of 25,600 cycles.
 * The full range of [0,255] is supported.
 */
#if    __18CXX
void Delay100TCYx(PARAM_SCLASS unsigned char);
#else /* 17CXX */
far void Delay100TCYx(PARAM_SCLASS unsigned char);
#endif

/* Delay1KTCYx
 * Delay multiples of 1000 Tcy
 * Passing 0 (zero) results in a delay of 256,000 cycles.
 * The full range of [0,255] is supported.
 */
#if    __18CXX
void Delay1KTCYx(PARAM_SCLASS unsigned char);
#else /* 17CXX */
far void Delay1KTCYx(PARAM_SCLASS unsigned char);
#endif

/* Delay10KTCYx
 * Delay multiples of 10,000 Tcy
 * Passing 0 (zero) results in a delay of 2,560,000 cycles.
 * The full range of [0,255] is supported.
 */
#if    __18CXX
void Delay10KTCYx(PARAM_SCLASS unsigned char);
#else /* 17CXX */
far void Delay10KTCYx(PARAM_SCLASS unsigned char);
#endif

#endif
