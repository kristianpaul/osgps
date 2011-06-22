#ifndef _TYPES_H
#define _TYPES_H 1

/**
 * A little wrapper to handle the varying support for stdint.h on
 * various platforms.
 **/

#if defined __USE_ISOC99
#include <stdint.h>
#elif (defined __KERNEL__)
#include <linux/types.h>
#else

/* Back-up for DOS */
/* Warning: these are NOT true for all architectures */
#ifndef __int8_t_defined
typedef char  int8_t;
typedef short int16_t;
typedef long  int32_t;

typedef unsigned char  uint8_t;
typedef unsigned short uint16_t;
typedef unsigned long  uint32_t;
#endif /* __int8_t_defined */

#endif /* defined __USE_ISOC99 */

#endif /* _TYPES_H */

