/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of 
 * which can be found via http://creativecommons.org (and should be included as 
 * LICENSE.txt within the associated archive or repository).
 */

#ifndef __HILEVEL_H
#define __HILEVEL_H

// Include functionality relating to newlib (the standard C library).

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include <string.h>

// Include functionality relating to the platform.

#include   "GIC.h"
#include   "MMU.h"
#include "PL011.h"

// Include functionality relating to the   kernel.

#include "lolevel.h"
#include     "int.h"

/* The kernel source code is made simpler via a type definition for
 * Page Table Entries (PTEs): for the selected parameterisation, it
 * suffices to model each PTE as a 32-bit value (although one could
 * argue use of bit-fields within a structure might simplify access
 * to constituent fields).
 */

typedef uint32_t pte_t;

#endif
