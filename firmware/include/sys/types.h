#ifndef VAYU_TYPES_H
#define VAYU_TYPES_H

typedef enum {
  NONE = 0,    // Invalid state recieved retry
  INVALID = 1, // Invalid state recieved retry
  ERROR = 2,   // Reinitialize and retry
  FAULT = 3,   // Kernel Panic (can't recover, reboot!)
  USAGE = 4,   // Incorrect usage don't retry
  EMPTY = 5    // No data to process
} err_t;

#endif //! VAYU_TYPES_H