#include <cstdlib>
#include <iostream>
#include <string>

int main(int argc, char* argv[]) {
  if (argc < 3) {
    std::cerr << "Usage: " << argv[0]
              << " <knob1=0|1> <input.mc> [ other args... ]\n";
    return 1;
  }

  int knob1 = std::atoi(argv[1]);
  // pick the right executable
  std::string cmd = knob1==1 ? "pipelined.exe" : "unpipelined.exe";

  // forward ONLY the filename + any additional args,
  // so start at argv[2], not argv[1]
  for (int i = 2; i < argc; ++i) {
    cmd += " ";
    cmd += argv[i];
  }

  return std::system(cmd.c_str());
}
