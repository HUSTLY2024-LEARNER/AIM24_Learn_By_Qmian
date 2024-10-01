#include <LangYa/CodeHelper.hpp>
#include <boost/program_options.hpp>

using namespace LangYa;

namespace {
}

int main(const int argc, const char* argv[]) {
	using namespace boost::program_options;

	options_description desc{"所有命令行选项"};
	variables_map variables{};

	std::string text{};
	desc.add_options()
		("text", value(&text), "一行文本")
		("help", "打印帮助信息");

	try {
		const auto parsed = parse_command_line(argc, argv, desc);
		store(parsed, variables);
	}
	catch (std::exception& ex) {
		std::cerr << ex.what() << '\n';
		return 1;
	}

	if (variables.contains("help") ){
		std::cout << desc << '\n';
		return 0;
	}

	variables.notify();

	std::cout << "text: " << text << '\n';
	return 0;
}
