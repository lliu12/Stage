#ifndef _CONNECTION_H
#define _CONNECTION_H

#include "stage.h"

typedef struct
{
  char* host; // client name
  int port;   // connection port

  int fd; // file descriptor. it matches one contained in the server's
	  // array of struct pollfds
  
  GPtrArray* subs; // array of subscriptions made by this client


  // etc.
  void* userdata; // hook for random data
} stg_connection_t;



void stg_connection_print( stg_connection_t* cli );
void stg_connection_print_cb( gpointer key, gpointer value, gpointer user );
stg_connection_t* stg_connection_create( void );

// close the connection and free the memory allocated
void stg_connection_destroy( stg_connection_t* con );

// are these used?
ssize_t stg_connection_write_msg( stg_connection_t* con, stg_msg_t* msg );
size_t stg_connection_read( stg_connection_t* con, void* buf, size_t len);
stg_msg_t* stg_connection_read_msg( stg_connection_t* con );

void stg_connection_sub_update( stg_connection_t* con );
void stg_connection_sub_update_cb( gpointer key, gpointer value, gpointer user );

#endif