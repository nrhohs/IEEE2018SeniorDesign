
#ifndef VL53L0X_EXPORT_H
#define VL53L0X_EXPORT_H

#ifdef VL53L0X_STATIC_DEFINE
#  define VL53L0X_EXPORT
#  define VL53L0X_NO_EXPORT
#else
#  ifndef VL53L0X_EXPORT
#    ifdef vl53l0x_EXPORTS
        /* We are building this library */
#      define VL53L0X_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define VL53L0X_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef VL53L0X_NO_EXPORT
#    define VL53L0X_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef VL53L0X_DEPRECATED
#  define VL53L0X_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef VL53L0X_DEPRECATED_EXPORT
#  define VL53L0X_DEPRECATED_EXPORT VL53L0X_EXPORT VL53L0X_DEPRECATED
#endif

#ifndef VL53L0X_DEPRECATED_NO_EXPORT
#  define VL53L0X_DEPRECATED_NO_EXPORT VL53L0X_NO_EXPORT VL53L0X_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef VL53L0X_NO_DEPRECATED
#    define VL53L0X_NO_DEPRECATED
#  endif
#endif

#endif
