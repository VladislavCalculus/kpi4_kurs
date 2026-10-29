#include <iostream>
#include <string_view>

int main(int argc, char* argv[]) {
    if (argc == 2 && std::string_view{argv[1]} == "--help") {
        std::cout << "Usage: kpi4_cli --help\n";
        return 0;
    }

    std::cerr << "Usage: kpi4_cli --help\n";
    return 2;
}
