#if defined(PINS_FINAL)
  #include "pins/final.h"
#elif defined(PINS_PROTOTYPE)
  #include "pins/prototype.h"
#elif defined(PINS_ITSYBITSY)
  #include "pins/itsybitsy.h"
#else
  #error "Pin assignment is not known."
#endif
