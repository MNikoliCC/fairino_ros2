// Prints the fault catalog as Markdown (docs/fairino_fault_recovery.md is
// generated from it so code and documentation cannot drift apart).
#include <iostream>

#include "fairino_cartesian_servo/fault_catalog.hpp"

int main() {
  std::cout << fairino_cartesian_servo::catalogMarkdown();
  return 0;
}
