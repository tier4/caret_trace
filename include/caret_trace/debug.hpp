#include <iostream>
#include <string>
#include <mutex>
#include <thread>

extern std::mutex debug_mtx;

class Debug
{
public:
  template<class Head, class ... Tail>
  void print(Head && head, Tail && ... tail)
  {
    std::lock_guard<std::mutex> lock(debug_mtx);
    std::cerr << std::this_thread::get_id();
    print_(head, std::forward<Tail>(tail)...);
    std::cerr << std::flush;
  }

private:
  void print_()
  {
    std::cerr << std::endl;
  }

  template<class Head, class ... Tail>
  void print_(Head && head, Tail && ... tail)
  {
    std::cerr << "," << head;
    print_(std::forward<Tail>(tail)...);
  }
};

extern Debug debug;
