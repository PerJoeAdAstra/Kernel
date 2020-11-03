/* Copyright (C) 2017 Daniel Page <csdsp@bristol.ac.uk>
 *
 * Use of this source code is restricted per the CC BY-NC-ND license, a copy of 
 * which can be found via http://creativecommons.org (and should be included as 
 * LICENSE.txt within the associated archive or repository).
 */

#include "hilevel.h"

/* T is a page table, which, for the 1MB pages (or sections) we use,
 * has 4096 entries: note the need to align T to a multiple of 16kB.
 */

pte_t T[ 4096 ] __attribute__ ((aligned (1 << 14)));

// Convenience pointers to base addresses of three pages in memory.

uint32_t* page_0x701 = ( uint32_t* )( 0x70110000 );
uint32_t* page_0x702 = ( uint32_t* )( 0x70210000 );
uint32_t* page_0x703 = ( uint32_t* )( 0x70310000 );

/* The following functions perform three, fairly limited tests based
 * on the initially configured MMU (i.e., wrt. the page table T).
 */

void test_0() {
  memset( page_0x701, 1, 1 << 20 );
  memset( page_0x702, 2, 1 << 20 );
  memset( page_0x703, 3, 1 << 20 );

  volatile uint32_t t1 = page_0x701[ 0 ];
  volatile uint32_t t2 = page_0x702[ 0 ];
  volatile uint32_t t3 = page_0x703[ 0 ];

  return;
}

void test_1() {
  pte_t t          = T[ 0x701 ];
        T[ 0x701 ] = T[ 0x702 ];
        T[ 0x702 ] = t         ;

  mmu_flush();

  volatile uint32_t t1 = page_0x701[ 0 ];
  volatile uint32_t t2 = page_0x702[ 0 ];
  volatile uint32_t t3 = page_0x703[ 0 ];

  return;
}

void test_2() {
  T[ 0x703 ] &= ~0x001E0; // mask domain
  T[ 0x703 ] |=  0x00020; // set  domain = 0001_{(2)} => client
  T[ 0x703 ] &= ~0x08C00; // mask access
  T[ 0x703 ] |=  0x00000; // set  access =  000_{(2)} => no access

  mmu_flush();

  volatile uint32_t t1 = page_0x701[ 0 ];
  volatile uint32_t t2 = page_0x702[ 0 ];
  volatile uint32_t t3 = page_0x703[ 0 ];

  return;
}

void hilevel_handler_rst() {
  // configure page table

  for( int i = 0; i < 4096; i++ ) {
    T[ i ] = ( ( pte_t )( i ) << 20 ) | 0x00C02;
  }

  // configure and enable MMU

  mmu_set_ptr0( T );

  mmu_set_dom( 0, 0x3 ); // set domain 0 to 11_{(2)} => manager (i.e., not checked)
  mmu_set_dom( 1, 0x1 ); // set domain 1 to 01_{(2)} => client  (i.e.,     checked)

  mmu_enable();

  // invoke test functions

  test_0();
  test_1();
  test_2();

  return;
}

void hilevel_handler_pab() {
  return;
}

void hilevel_handler_dab() {
  return;
}
