using namespace std;

void* operator new(size_t size);
void* operator new[](size_t size);
void operator delete(void *p);
void operator delete[](void *p);

void* operator new(size_t size, bool);
void* operator new[](size_t size, bool);
//void operator delete(void *p, bool);
//void operator delete[](void *p, bool);
