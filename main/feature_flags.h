#pragma once

// Build-time protocol features for cp705.
//
// Override at CMake time:
//   idf.py build -DENABLE_FT4=OFF

// Include FT4 protocol code and the runtime Mode menu item. When disabled,
// only FT8 is available.
#ifndef ENABLE_FT4
#define ENABLE_FT4 1
#endif
