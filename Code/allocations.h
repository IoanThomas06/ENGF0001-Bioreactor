#include <stdlib.h>

void *checkedMalloc(size_t size, char errorMessage[]);
void *checkedCalloc(size_t numberOfObjects, size_t size, char errorMessage[]);