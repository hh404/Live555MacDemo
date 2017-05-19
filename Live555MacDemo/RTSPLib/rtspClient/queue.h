
#ifndef QUEUE_H
#define QUEUE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <pthread.h>
#include <stdint.h>
    
typedef struct AVPacket {
    uint8_t *data;
    int   size;
} AVPacket;
typedef struct _sPARAMS{
	unsigned long ulLength;
	unsigned long ulWidth;
	unsigned long ulHeight;
	unsigned long ulFrameRate;
	unsigned long ulBitRate;
}sPARMAS;//the queue includes params info



typedef struct node {
    struct node* last;
    struct node* next;
    void* data;
}node_t;

typedef struct queue {
    pthread_mutex_t mutex;
    pthread_cond_t condv;
    node_t* head;
    int size;
}queue_t;

int q_init(queue_t** q);
int q_push_head(queue_t* q, void* data);
int q_push_tail(queue_t* q, void* data);
void* q_pop_head(queue_t* q);
void* q_pop_tail(queue_t* q);
int q_size(queue_t* q);
void q_abort(queue_t* q, void (*func)(void*));
void q_wake(queue_t* q);
void q_free(queue_t** q, void (*func)(void*));

int queue_init();
void queue_free();

void free_AVPacket(void* data);

#define DECLARE_QUEUE(NAME, TYPE) \
    int NAME##_init(); \
    void NAME##_free(); \
    int NAME##_push_head(TYPE* elm); \
    int NAME##_push_tail(TYPE* elm); \
    TYPE* NAME##_pop_head(); \
    TYPE* NAME##_pop_tail(); \
    int NAME##_size(); \
    void NAME##_abort(); \
    void NAME##_wake();

#define IMPLEMENT_QUEUE(NAME, TYPE) \
    static queue_t* NAME = 0; \
    int NAME##_init() { \
        if (NAME) \
            return -1; \
        return q_init(&NAME); \
    } \
    void NAME##_free() { \
        if (NAME) { \
            q_free(&NAME, free_##TYPE); \
        } \
    } \
    int NAME##_push_head(TYPE* elm) { \
        if (!NAME) \
            return -1; \
        return q_push_head(NAME, elm); \
    } \
    int NAME##_push_tail(TYPE* elm) { \
        if (!NAME) \
            return -1; \
        return q_push_tail(NAME, elm); \
    } \
    TYPE* NAME##_pop_head() { \
        if (!NAME) \
            return 0; \
        return (TYPE*) q_pop_head(NAME); \
    } \
    TYPE* NAME##_pop_tail() { \
        if (!NAME) \
            return 0; \
        return (TYPE*) q_pop_tail(NAME); \
    } \
    int NAME##_size() { \
        if (!NAME) \
            return -1; \
        return q_size(NAME); \
    } \
    void NAME##_abort() { \
        if (!NAME) \
            return; \
        q_abort(NAME, free_##TYPE); \
    } \
    void NAME##_wake() { \
        if (!NAME) \
            return; \
        q_wake(NAME); \
    }

DECLARE_QUEUE(audio_packet_queue, AVPacket)
DECLARE_QUEUE(video_packet_queue, AVPacket)
DECLARE_QUEUE(subtitle_packet_queue, AVPacket)

#ifdef __cplusplus
}
#endif


#endif

