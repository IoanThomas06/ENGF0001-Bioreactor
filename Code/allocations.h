#include <stdlib.h>

#ifndef ALLOCATIONS
#define ALLOCATIONS

void *checkedMalloc(size_t, char[]);
void *checkedCalloc(size_t, size_t, char[]);

#endif