/*
 * globals.h
 *
 *  Created on: Apr 3, 2019
 *      Author: quzhi
 */

#ifndef GLOBALS_H_
#define GLOBALS_H_

#include "sharedmemory.h"


shared_memory::shared_memory small_buffer(16, 32);
shared_memory::shared_memory large_buffer(256, 4);


#endif /* GLOBALS_H_ */
