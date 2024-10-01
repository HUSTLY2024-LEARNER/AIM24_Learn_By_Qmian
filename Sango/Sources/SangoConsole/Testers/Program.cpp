#include <list>
#include <string>

#include <LangYa/CodeHelper.hpp>

using namespace LangYa;

namespace {
	template<typename TUserCommand, typename TSystemResponse>
	struct ConsoleItem {
		using UserCommandType = TUserCommand;
		using SystemResponseType = TSystemResponse;

		TUserCommand UserCommand{};
		std::list<TSystemResponse> SystemResponseList{};
	};

	template<typename TUserCommand, typename TSystemResponse, typename TConsoleItem = ConsoleItem<TUserCommand, TSystemResponse>>
	struct SimpleConsole {
		using ConsoleItemType = TConsoleItem;

		std::list<TConsoleItem> ItemList{};
		std::function<void(TConsoleItem& item)> WhenNewCommand{};
		std::function<void(const TConsoleItem& item)> WhenNewCommandFinished{};

		void NewCommand(const TUserCommand& command) {
			ItemList.emplace_back();
			auto& item = ItemList.back();
			item.UserCommand = command;
			if (WhenNewCommand) WhenNewCommand(item);
			if (WhenNewCommandFinished) WhenNewCommandFinished(item);
		}
	};
}

int main() {
	using SangoConsole = SimpleConsole<std::string, std::string>;

	SangoConsole console{};
	return 0;
}
