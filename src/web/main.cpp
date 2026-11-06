#include <cstdio>
#include <exception>
#include <iostream>
#include <string>

#include "irmv/web/record_query.hpp"

// The stdin/stdout JSONL bridge of the experiment web service. One request per
// line in, one reply line out; stdout carries protocol only and every
// diagnostic goes to stderr. A malformed request is answered on stdout like any
// other request, so a service never has to distinguish a dead helper from a
// refused query by the stream's silence.
int main(int argc, char **argv) {
  std::string records_root;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--records-root" && index + 1 < argc) {
      records_root = argv[++index];
      continue;
    }
    std::fprintf(stderr,
                 "usage: web_record_query --records-root PATH\n"
                 "unexpected argument: %s\n",
                 argument.c_str());
    return 2;
  }
  if (records_root.empty()) {
    std::fprintf(stderr, "usage: web_record_query --records-root PATH\n");
    return 2;
  }

  try {
    irmv::web::RecordQuery query(records_root);
    std::string line;
    while (std::getline(std::cin, line)) {
      if (line.empty()) {
        continue;
      }
      std::cout << query.Execute(line) << '\n';
      std::cout.flush();
    }
  } catch (const std::exception &error) {
    std::fprintf(stderr, "web_record_query: %s\n", error.what());
    return 1;
  }
  return 0;
}
