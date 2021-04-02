#include <mbed.h>

enum class DataType{

};
class Communication{
    BufferedSerial dev;
      void send_data(char * buf, size_t length){
      dev.write(buf,  length);
    };
    void on_rcv_data(char * buf, int size){

    };

};