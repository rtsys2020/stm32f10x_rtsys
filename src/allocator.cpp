/*
 * allocator.c
 *
 *  Created on: Jul 14, 2015
 *      Author: mohammadreza
 */

//#include <iostream>

using namespace std;

#include "sdmalloc.h"

void* operator new(size_t size) {
//	cout << "operator new(" << size << "):\t";
	void *p;
	if (size > 128)
		p = sdpvPortMalloc(size);
	else
		p = pvPortMalloc(size);

//	if (!p)
//		throw "operator new() error";
//	cout << static_cast<void*>(p) << endl;
//	if (!p)
//		std::cout << "new operator failed!" << std::endl;
	return p;
}

void* operator new[](size_t size) {
//	cout << "operator new[](" << size << "):\t";
	void *p;
	if (size > 128)
		p = sdpvPortMalloc(size);
	else
		p = pvPortMalloc(size);
//	if (!p)
//		throw "operator new() error";
//	cout << static_cast<void*>(p) << endl;
//	if (!p)
//		std::cout << "new[] operator failed!" << std::endl;
	return p;
}

void operator delete(void *p) {
//	cout << "operator delete(" << p << ")" << endl;
	if (p) {
		if ((uint32_t) p >= 0xc0000000)
			sdvPortFree(p);
		else
			vPortFree(p);
	}
//	} else
//		std::cout << "memory already freed!" << std::endl;
}

void operator delete[](void *p) {
//	cout << "operator delete[](" << p << ")" << endl;
	if (p) {
		if ((uint32_t) p >= 0xc0000000)
			sdvPortFree(p);
		else
			vPortFree(p);
	}
//	else
//		std::cout << "memory already freed!" << std::endl;
}

//////SDRAM Heap\\\\\\\\

void* operator new(size_t size,bool internal) {
//	cout << "operator new(" << size << "):\t";
	void *p;
	if (internal)
		p = pvPortMalloc(size);
	else
		p = sdpvPortMalloc(size);
//	if (!p)
//		throw "operator new() error";
//	cout << static_cast<void*>(p) << endl;
//	if (!p)
//		std::cout << "new sd operator failed!" << std::endl;
	return p;
}

void* operator new[](size_t size, bool internal) {
//	cout << "operator new[](" << size << "):\t";
	void *p;
	if (internal)
		p = pvPortMalloc(size);
	else
		p = sdpvPortMalloc(size);
//	if (!p)
//		throw "operator new() error";
//	cout << static_cast<void*>(p) << endl;
//	if (!p)
//		std::cout << "new sd[] operator failed!" << std::endl;
	return p;
}

//void operator delete(void *p,bool b) {
////	cout << "operator delete(" << p << ")" << endl;
//	if (p)
//		sdvPortFree(p);
//	else
//		std::cout << "sd memory already freed!" << std::endl;
//}
//
//void operator delete[](void *p,bool b) {
////	cout << "operator delete[](" << p << ")" << endl;
//	if (p)
//		sdvPortFree(p);
//	else
//		std::cout << "sd memory already freed!" << std::endl;
//}
