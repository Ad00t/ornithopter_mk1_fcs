#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

#define DEFINE_QUEUE(name, type, max_size)                       	\
typedef struct {                                                     \
    type data[max_size];                                            \
    int head;                                                        \
    int tail;                                                        \
    int count;                                                       \
    type (*copy_fn)(type* val);                                     \
    void  (*free_fn)(type* val);                                     \
} name;                                                               \
                                                                      \
static inline void name##_Init(name* q,                                \
                               type (*copy_fn)(type),               \
                               void (*free_fn)(type)) {              \
    q->head = q->tail = q->count = 0;                                 \
    q->copy_fn = copy_fn;                                             \
    q->free_fn = free_fn;                                             \
}                                                                     \
                                                                      \
static inline bool name##_IsFull(name* q) {                            \
    return q->count == max_size;                                      \
}                                                                     \
                                                                      \
static inline bool name##_IsEmpty(name* q) {                           \
    return q->count == 0;                                             \
}                                                                     \
                                                                      \
static inline bool name##_Enqueue(name* q, type val) {               \
    if (name##_IsFull(q)) return false;                                \
    if (q->copy_fn)                                                   \
        q->data[q->tail] = q->copy_fn(val);                           \
    else                                                              \
        q->data[q->tail] = val;                                       \
    q->tail = (q->tail + 1) % max_size;                               \
    q->count++;                                                        \
    return true;                                                       \
}                                                                     \
                                                                      \
static inline bool name##_Dequeue(name* q, type* val) {              \
    if (name##_IsEmpty(q)) return false;                               \
    *val = q->data[q->head];                                          \
    q->head = (q->head + 1) % max_size;                               \
    q->count--;                                                        \
    return true;                                                       \
}                                                                     \
                                                                      \
static inline void name##_Clear(name* q) {                             \
    if (q->free_fn) {                                                 \
        for (int i = 0; i < q->count; i++) {                          \
            int idx = (q->head + i) % max_size;                       \
            q->free_fn(q->data[idx]);                                  \
        }                                                              \
    }                                                                  \
    q->head = q->tail = q->count = 0;                                 \
}
