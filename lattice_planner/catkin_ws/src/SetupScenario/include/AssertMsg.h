#pragma once
#ifndef __ASSERT_MSG_H__
#define __ASSERT_MSG_H__

#include <iostream>
#include <string>
#include <sstream>

#ifndef NDEBUG
  #define AssertMsg(expression, message) \
          { std::stringstream ss; \
            ss << message; \
            __assert_msg(#expression, expression, __FILE__, __LINE__, ss.str()); \
          }
          
  #define Assert(expression) \
          __assert(#expression, expression, __FILE__, __LINE__)
          
#else
  #define AssertMsg(expression, message) ;
  #define AssertMsg(expression) ;
#endif

inline void __assert_msg(const char * expression_string, 
                         bool expression, 
                         const char * file_name, 
                         int line,
                         std::string message) {
  if(!expression) {
    std::cerr << "Assert failed:\t" << expression_string << "\n"
              << "  Info:       \t" << message << "\n"
              << "  Source:     \t" << file_name << ", line " << line << "\n";
    abort();
  }
}

inline void __assert(const char * expression_string, 
                     bool expression, 
                     const char * file_name, 
                     int line) {
  if(!expression) {
    std::cerr << "Assert failed:\t" << expression_string << "\n"
              << "  Source:     \t" << file_name << ", line " << line << "\n";
    abort();
  }
}


#endif
