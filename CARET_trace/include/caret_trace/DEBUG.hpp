#pragma once

#include "caret_trace/tracing_controller.hpp"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <shared_mutex>
static std::shared_mutex smtx; // 共有ミューテックスの宣言

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
  std::shared_lock<std::shared_mutex> lock(smtx); \
  std::ostringstream buf; \
  void* callstack[2]; \
  int frames = backtrace(callstack, 2); \
  char** symbols = backtrace_symbols(callstack, frames); \
  char fn[512]; \
  extractc_fn(symbols[1], fn, 512); \
  buf << std::setbase(10) << getpid() << "/ " << gettid() << \
        ": [" << fn << "->" <<__func__ << "] " << __LINE__ << ": " << \
        std::setbase(16) << std::uppercase << #X << "=" << X << std::endl; \
  std::cout << buf.str(); \
  free(symbols); \
}

#define GET_PREV(S) { \
  std::shared_lock<std::shared_mutex> lock(smtx); \
  std::ostringstream buf; \
  void* callstack[2]; \
  int frames = backtrace(callstack, 2); \
  char** symbols = backtrace_symbols(callstack, frames); \
  char fn[512]; \
  extractc_fn(symbols[1], fn, 512); \
  std::cout << buf.str(); \
  free(symbols); \
  S = fn; \
}

#define D(X) { \
          std::shared_lock<std::shared_mutex> lock(smtx); \
          std::ostringstream buf; \
          buf << std::setbase(10) << getpid() << "/ " << gettid() << ": "; \
          buf << __func__ << ": " << __LINE__ << " | " << \
          std::setbase(16) << std::uppercase << #X << "=" << X << std::endl; \
          std::cout << buf.str(); \
        }

#define D_IGN(TYPE, KEY, X, TRC) { \
          std::shared_lock<std::shared_mutex> lock(smtx); \
          std::ostringstream buf; \
          buf << std::setbase(10) << getpid() << "/ " << gettid() << ": "; \
          buf << "--- IGNORED " << __func__ << ": " << __LINE__ << \
          " [" << #TRC << "] " << std::setbase(16) << std::uppercase << #KEY << "=" << KEY << " " << \
          "NODE=" << context.get_controller().get_node_name(TYPE, KEY) << " " << #X << "=" << X << std::endl; \
          std::cout << buf.str(); \
        }

#define D_SEL(TYPE, KEY, X, TRC) { \
          std::shared_lock<std::shared_mutex> lock(smtx); \
          std::ostringstream buf; \
          buf << std::setbase(10) << getpid() << "/ " << gettid() << ": "; \
          buf << "+++ SELECTED " << __func__ << ": " << __LINE__ << \
          " [" << #TRC << "] " << std::setbase(16) << std::uppercase << #KEY << "=" << KEY << " " << \
          "NODE=" << context.get_controller().get_node_name(TYPE, KEY) << " " << #X << "=" << X << std::endl; \
          std::cout << buf.str(); \
        }

#define D_IGN_ONCE(TYPE, KEY, X, TRC) { \
          std::shared_lock<std::shared_mutex> lock(smtx); \
          std::ostringstream buf; \
          static int once = 0; \
          if (once < 5) { \
            buf << std::setbase(10) << getpid() << "/ " << gettid() << ": "; \
            buf << "--- IGNORED_ONCE " << __func__ << ": " << __LINE__ << \
            " [" << #TRC << "] " << std::setbase(16) << std::uppercase << #KEY << "=" << KEY << " " << \
            "NODE=" << context.get_controller().get_node_name(TYPE, KEY) << " " << #X << "=" << X << std::endl; \
            std::cout << buf.str(); \
            once++; \
          } \
        }

#define D_SEL_ONCE(TYPE, KEY, X, TRC) { \
          std::shared_lock<std::shared_mutex> lock(smtx); \
          std::ostringstream buf; \
          static int once = 0; \
          if (once < 5) { \
            buf << std::setbase(10) << getpid() << "/ " << gettid() << ": "; \
            buf << "+++ SELECTED_ONCE " << __func__ << ": " << __LINE__ << \
            " [" << #TRC << "] " << std::setbase(16) << std::uppercase << #KEY << "=" << KEY << " " << \
            "NODE=" << context.get_controller().get_node_name(TYPE, KEY) << " " << #X << "=" << X << std::endl; \
            std::cout << buf.str(); \
            once++; \
          } \
        }

#define DIF(cond1, cond2) { \
          std::shared_lock<std::shared_mutex> lock(smtx); \
          std::ostringstream buf; \
          buf << std::setbase(10) << getpid() << "/ " << gettid() << ": "; \
          if (!(cond1 && cond2)) { \
            buf << "NOT REGISTED " << __func__ << ": " << __LINE__ << " "; \
            if (!cond1) buf << #cond1 << "=" << cond1 << " "; \
            if (!cond2) buf << #cond2 << "=" << cond2; \
            buf << std::endl; \
          } else { \
            buf << "REGISTED " << __func__ << ": " << __LINE__ << " "; \
            buf << #cond1 << "=" << cond1 << " "; \
            buf << #cond2 << "=" << cond2; \
            buf << std::endl; \
          } \
          std::cout << buf.str(); \
        }

#define DIF_ONCE(cond1, cond2) { \
          std::shared_lock<std::shared_mutex> lock(smtx); \
          std::ostringstream buf; \
          static int once1 = 0; \
          static int once2 = 0; \
          buf << std::setbase(10) << getpid() << "/ " << gettid() << ": "; \
          if (!(cond1 && cond2)) { \
            if (once1 < 5) { \
              buf << "NOT REGISTED (ONCE) " << __func__ << ": " << __LINE__ << " "; \
              if (!cond1) buf << #cond1 << "=" << cond1 << " "; \
              if (!cond2) buf << #cond2 << "=" << cond2; \
              buf << std::endl; \
              once1++; \
            } \
          } else { \
            if (once2 < 5) { \
              buf << "REGISTED (ONCE) " << __func__ << ": " << __LINE__ << " "; \
              buf << #cond1 << "=" << cond1 << " "; \
              buf << #cond2 << "=" << cond2; \
              buf << std::endl; \
              once2++; \
            } \
          } \
          std::cout << buf.str(); \
        }

#define SEL 1
