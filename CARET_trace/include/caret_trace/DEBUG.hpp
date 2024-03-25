#pragma once

#include "caret_trace/tracing_controller.hpp"

#include <execinfo.h>
#ifdef __cplusplus
  #include <cstring>
  #else
  #include <string.h>
  #endif
#include <unistd.h>
static void extractc_fn(const char* symbol, char* fn, size_t bufSize) {
    const char* start = strchr(symbol, '(');
    if (start) {
        ++start; // '('を飛ばす
        const char* end = strchr(start, '+');
        if (end) {
            strncpy(fn, start, end - start);
            fn[end - start] = '\0'; // 終端文字を追加
        } else {
            strncpy(fn, start, bufSize - 1);
            fn[bufSize - 1] = '\0';
        }
    } else {
        strncpy(fn, symbol, bufSize - 1);
        fn[bufSize - 1] = '\0';
    }
}

#define DS(X) { \
  void* callstack[2]; \
  int frames = backtrace(callstack, 2); \
  char** symbols = backtrace_symbols(callstack, frames); \
  char fn[512]; \
  extractc_fn(symbols[1], fn, 512); \
  std::cout << std::setbase(10) << getpid() << "/ " << gettid() << \
        ": [" << fn << "->" <<__func__ << "] " << __LINE__ << ": " << \
        std::setbase(16) << std::uppercase << #X << "=" << X << std::endl; \
  free(symbols); \
}

#define GET_PREV(S) { \
  void* callstack[2]; \
  int frames = backtrace(callstack, 2); \
  char** symbols = backtrace_symbols(callstack, frames); \
  char fn[512]; \
  extractc_fn(symbols[1], fn, 512); \
  free(symbols); \
  S = fn; \
}

#define D(X) {std::cout << std::setbase(10) << __func__ << ": " << __LINE__ << " | " << \
            std::setbase(16) << std::uppercase << #X << "=" << X << std::endl;}

#define D_NH(TYPE, KEY, X, TRC) {std::cout << std::setbase(10) << "FILTERED " << __func__ << ": " << __LINE__ << \
            " [" << #TRC << "] " << std::setbase(16) << std::uppercase << #KEY << "=" << KEY << " " << \
            "NODE=" << context.get_controller().get_node_name(TYPE, KEY) << " " << #X << "=" << X << std::endl;}


#if 0
#define DIF(cond1, cond2) { \
              if (!(cond1 && cond2)) { \
                std::cout << std::setbase(10) << "--ATTENTION " << __func__ << ": " << __LINE__ << " "; \
                if (!cond1) std::cout << #cond1 << "=" << cond1 << " "; \
                if (!cond2) std::cout << #cond2 << "=" << cond2; \
                std::cout << std::endl; \
              } \
            }
#endif
#define DIF(cond1, cond2) { \
              if (!(cond1 && cond2)) { \
                std::cout << std::setbase(10) << "--ATTENTION " << __func__ << ": " << __LINE__ << " "; \
                if (!cond1) std::cout << #cond1 << "=" << cond1 << " "; \
                if (!cond2) std::cout << #cond2 << "=" << cond2; \
                std::cout << std::endl; \
              } else { \
                std::cout << std::setbase(10) << "++OK " << __func__ << ": " << __LINE__ << " "; \
                std::cout << #cond1 << "=" << cond1 << " "; \
                std::cout << #cond2 << "=" << cond2; \
                std::cout << std::endl; \
              } \
            }

#define SEL 1
