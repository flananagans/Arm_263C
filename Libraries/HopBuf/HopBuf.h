#ifndef HOPBUF_H
#define HOPBUF_H

#include <Arduino.h>

#define HOPBUFSIZE 64

/*
 * Simple FIFO buffer template class
 */
template<typename DataType>
class HopBuf {

    public:
        uint8_t len = 0;
        long missed = 0;

        HopBuf(){}

        bool append(DataType* d);
        bool pop(DataType* dest);
        bool peek(DataType* dest, uint8_t idx);

    private:
        DataType buf[HOPBUFSIZE];
        volatile uint8_t head = 0;
        volatile uint8_t tail = 0;

};

/*
 * Add data to the end of the buffer
 */
template<typename DataType>
bool HopBuf<DataType>::append(DataType* d) {

    memcpy(&this->buf[this->tail], d, sizeof(DataType));
    this->tail = ++this->tail % HOPBUFSIZE;
    if(this->len == HOPBUFSIZE) {
        this->head = ++this->head % HOPBUFSIZE;
        this->missed++;
        return false;
    } else {
        this->len++;
        return true;
    }
}

/*
 * pop the oldest element
 */
template<typename DataType>
bool HopBuf<DataType>::pop(DataType* dest) {
    if(this->len == 0) {
        return false;
    }
    
    memcpy(dest, &this->buf[this->head], sizeof(DataType));
    this->head = ++this->head % HOPBUFSIZE;
    this->len--;

    return true;
}

/*
 * peek an element
 */
template<typename DataType>
bool HopBuf<DataType>::peek(DataType* dest, uint8_t idx) {
    if(idx >= this->len) {
        return false;
    }
    
    memcpy(dest, &this->buf[(this->head + idx) % HOPBUFSIZE], sizeof(DataType));

    return true;
}
#endif
        
