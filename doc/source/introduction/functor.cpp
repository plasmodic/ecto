#include <iostream>
#include <string>
//start
struct Printer
{
  Printer(const std::string& prefix, const std::string& postfix)
      :
        prefix_(prefix),
        postfix_(postfix)
  {
  }
  void
  operator()(std::ostream& out, const std::string& message)
  {
    out << prefix_ << message << postfix_;
  }
  std::string prefix_, postfix_;
};
//end

int
main()
{
  Printer p("begin>>", "<<end\n");
  while (true)
  {
    std::string word;
    std::cin >> word;
    p(std::cout, word);
  }
  return 0;
}
