/* Automatically generated nanopb header */
/* Generated by nanopb-0.4.5 */

#ifndef PB_COMMON_PB_H_INCLUDED
#define PB_COMMON_PB_H_INCLUDED
#include "pb.h"

#if PB_PROTO_HEADER_VERSION != 40
#error Regenerate this file with the current version of nanopb generator.
#endif

/* Enum definitions */
typedef enum _InterfaceControlMode { 
    InterfaceControlMode_NONE = 0, 
    InterfaceControlMode_HIGH = 1, 
    InterfaceControlMode_LOW = 2 
} InterfaceControlMode;

/* Helper constants for enums */
#define _InterfaceControlMode_MIN InterfaceControlMode_NONE
#define _InterfaceControlMode_MAX InterfaceControlMode_LOW
#define _InterfaceControlMode_ARRAYSIZE ((InterfaceControlMode)(InterfaceControlMode_LOW+1))


#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
