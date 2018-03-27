/*
 * Log.h
 *
 *  Created on: 22 mars 2018
 *      Author: Guillaume
 */

#ifndef TOOLS_LOG_H_
#define TOOLS_LOG_H_

#ifdef DEBUG
#define LOG_DEBUG(x, ...)	printf(x, __VA_ARGS__)
#else
#define LOG_DEBUG(x, ...)
#endif

#endif /* TOOLS_LOG_H_ */
