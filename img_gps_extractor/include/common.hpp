/* 
 * common.hpp
 * 
 * Basic functions common to all files (or most).
 *
 * May 2017 - Andres Milioto
 * 
 */
#ifndef _COMMON_H_
#define _COMMON_H_

#include <stdio.h>

#ifdef _DEBUG
//debug tracer (prints stuff out in console besides generating yaml)
#define DEBUG(format, ...) do {          \
    if (_verbose)                        \
        printf(format, ##__VA_ARGS__);   \
    }while(0)
#else
#define DEBUG(format, ...)
#endif


#endif // _COMMON_H_
