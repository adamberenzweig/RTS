/*
 * MemoryFree.c
 * returns the number of free RAM bytes
 */

#include "WProgram.h"  
#include "MemoryFree.h"

// Uncomment for more verbose memory profile.
//#define MEM_PROFILE

extern unsigned int __data_start;
extern unsigned int __data_end;
extern unsigned int __bss_start;
extern unsigned int __bss_end;
extern unsigned int __heap_start;
extern void *__brkval;

int freeMemory()
{
  int free_memory;

  if((int)__brkval == 0)
     free_memory = ((int)&free_memory) - ((int)&__bss_end);
  else
    free_memory = ((int)&free_memory) - ((int)__brkval);

  return free_memory;
}

// RAMEND and SP seem to be available without declaration here
int16_t ramSize=0;   // total amount of ram available for partitioning
int16_t dataSize=0;  // partition size for .data section
int16_t bssSize=0;   // partition size for .bss section
int16_t heapSize=0;  // partition size for current snapshot of the heap section
int16_t stackSize=0; // partition size for current snapshot of the stack section
int16_t freeMem1=0;  // available ram calculation #1
int16_t freeMem2=0;  // available ram calculation #2

//* This function places the current value of the heap and stack pointers in the
// * variables. You can call it from any place in your code and save the data for
// * outputting or displaying later. This allows you to check at different parts of
// * your program flow.
// * The stack pointer starts at the top of RAM and grows downwards. The heap pointer
// * starts just above the static variables etc. and grows upwards. SP should always
// * be larger than HP or you'll be in big trouble! The smaller the gap, the more
// * careful you need to be. Julian Gall 6-Feb-2009.
// *
uint8_t *heapptr, *stackptr;
uint16_t diff=0;
void check_mem() {
  stackptr = (uint8_t *)malloc(4);          // use stackptr temporarily
  heapptr = stackptr;                     // save value of heap pointer
  free(stackptr);      // free up the memory again (sets stackptr to 0)
  stackptr =  (uint8_t *)(SP);           // save value of stack pointer
}

// Adapted from:
// http://www.arduino.cc/playground/Main/CorruptArrayVariablesAndMemory
void memrep() {
  //Serial.print("\n---mem---\n");
  Serial.print("free: ");
  Serial.print( freeMemory() );

#ifdef MEM_PROFILE
  // Serial.print("\n\nSP should always be larger than HP or you'll be in big trouble!");
  check_mem();
  /*
  Serial.print("\nHP=[0x"); Serial.print( (int) heapptr, HEX);
  Serial.print("] (grows+)");
  // Serial.print( (int) heapptr, DEC); Serial.print(" decimal)");

  Serial.print("\nSP=[0x"); Serial.print( (int) stackptr, HEX);
  Serial.print("] (grows-)");
  //Serial.print( (int) stackptr, DEC); Serial.print(" decimal)");
  */

  //Serial.print("\ndifference should be positive: diff=stackptr-heapptr, diff=[0x");
  Serial.print("\nSP-HP=");
  diff=stackptr-heapptr;
  Serial.print( (int) diff, DEC); //  Serial.print("] (bytes decimal)");

  /*
  Serial.print("\nEND free: ");
  Serial.print( freeMemory() );
  */

  // ---------------- Print memory profile -----------------
  /*
  Serial.print("\n__data_start=0x"); Serial.print( (int) &__data_start, HEX );
  //Serial.print("] which is ["); Serial.print( (int) &__data_start, DEC); Serial.print("] bytes decimal");

  Serial.print("\n__data_end=0x"); Serial.print((int) &__data_end, HEX );
  //Serial.print("] which is ["); Serial.print( (int) &__data_end, DEC); Serial.print("] bytes decimal");

  Serial.print("\n__bss_start=0x"); Serial.print((int) & __bss_start, HEX );
  //Serial.print("] which is ["); Serial.print( (int) &__bss_start, DEC); Serial.print("] bytes decimal");

  Serial.print("\n__bss_end=0x"); Serial.print( (int) &__bss_end, HEX );
  //Serial.print("] which is ["); Serial.print( (int) &__bss_end, DEC); Serial.print("] bytes decimal");

  Serial.print("\n__heap_start=0x"); Serial.print( (int) &__heap_start, HEX );
  //Serial.print("] which is ["); Serial.print( (int) &__heap_start, DEC); Serial.print("] bytes decimal");

  Serial.print("\n__malloc_heap_start=0x");
  Serial.print( (int) __malloc_heap_start, HEX );
  //Serial.print("] which is ["); Serial.print( (int) __malloc_heap_start, DEC); Serial.print("] bytes decimal");

  Serial.print("\n__malloc_margin=0x");
  Serial.print( (int) &__malloc_margin, HEX );
  //Serial.print("] which is ["); Serial.print( (int) &__malloc_margin, DEC); Serial.print("] bytes decimal");

  Serial.print("\n__brkval=0x"); Serial.print( (int) __brkval, HEX );
  //Serial.print("] which is ["); Serial.print( (int) __brkval, DEC); Serial.print("] bytes decimal");

  Serial.print("\nSP=0x"); Serial.print( (int) SP, HEX );
  //Serial.print("] which is ["); Serial.print( (int) SP, DEC); Serial.print("] bytes decimal");

  Serial.print("\nRAMEND=0x"); Serial.print( (int) RAMEND, HEX );
  //Serial.print("] which is ["); Serial.print( (int) RAMEND, DEC); Serial.print("] bytes decimal");
  */

  // summaries:
  ramSize   = (int) RAMEND       - (int) &__data_start;
  dataSize  = (int) &__data_end  - (int) &__data_start;
  bssSize   = (int) &__bss_end   - (int) &__bss_start;
  heapSize  = (int) __brkval     - (int) &__heap_start;
  stackSize = (int) RAMEND       - (int) SP;
  freeMem1  = (int) SP           - (int) __brkval;
  freeMem2  = ramSize - stackSize - heapSize - bssSize - dataSize;
  //Serial.print("\n--- section size summaries ---");
  Serial.print("\nram  = "); Serial.print( ramSize, DEC );
  Serial.print("\n.data= "); Serial.print( dataSize, DEC );
  Serial.print("\n.bss = "); Serial.print( bssSize, DEC );
  Serial.print("\nheap = "); Serial.print( heapSize, DEC );
  Serial.print("\nstack= "); Serial.print( stackSize, DEC );
  Serial.print("\nfree= "); Serial.print( freeMem1, DEC );
  Serial.print("\nfree= "); Serial.print( freeMem2, DEC );
  Serial.print("\n----");
#endif

  Serial.println();
}
