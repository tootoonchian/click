#ifndef HASHDEMUX_HH
#define HASHDEMUX_HH
#include "element.hh"

/*
 * =c
 * HashDemux(OFFSET, LENGTH)
 * =d
 * Can have any number of outputs.
 * Chooses the output on which to emit each packet based on
 * a hash of the LENGTH bytes starting at OFFSET.
 * Could be used for stochastic fair queuing.
 * =e
 * This element expects IP packets and chooses the output
 * based on a hash of the IP destination address:
 * 
 * = HashDemux(16, 4)
 */

class HashDemux : public Element {

  int _offset;
  int _length;
  
 public:
  
  HashDemux();
  
  const char *class_name() const		{ return "HashDemux"; }
  const char *processing() const		{ return PUSH; }
  void notify_noutputs(int);
  
  HashDemux *clone() const;
  int configure(const String &, ErrorHandler *);
  
  void push(int port, Packet *);
  
};

#endif
