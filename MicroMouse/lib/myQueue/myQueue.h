#ifndef MYQUEUE_H
#define MYQUEUE_H

#include <stdio.h>
#include <stdlib.h>

typedef struct _myQueue* myQueue;
myQueue queue_create();
void queue_destroy(myQueue q);
void queue_push(myQueue q, int elem);
int queue_pop(myQueue q);
int queue_first(myQueue q);
int queue_is_empty(myQueue q);
int queue_size(myQueue q);
void queue_clear(myQueue q);

struct node {
    int data;
    struct node* next;
};

struct _myQueue {
    struct node* head;
    struct node* tail;
    int size;
};

#endif