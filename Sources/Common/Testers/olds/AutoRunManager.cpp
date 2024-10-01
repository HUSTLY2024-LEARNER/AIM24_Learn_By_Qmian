#include <LangYa/CodeHelper.hpp>

using namespace LangYa;

namespace {
	using path_t = boost::filesystem::path;

	struct ScriptInfo {
		path_t Path{};
		bool IsEnabled{};

		bool TryFlip() {
			const auto old_path = Path;
			IsEnabled = !IsEnabled;
			Path.replace_extension(IsEnabled ? ".enabled" : ".disabled");
			try { rename(old_path, Path); }
			catch (const std::exception& e) {
				spdlog::error("在修改文件({})的状态时发生异常: {}", Path.string(), e.what());
				return false;
			}
			return true;
		}

		// ReSharper disable once CppDeclaratorNeverUsed
		std::ostream& Format(std::ostream& stream) const noexcept {
			return stream << (IsEnabled ? "[x] " : "[ ] ") << Path;
		}
	};

	using ScriptList = std::vector<ScriptInfo>;

	bool AcquireScriptList(const path_t& directory, ScriptList& scripts) {
		if (!exists(directory)) {
			spdlog::error("找不到目标文件夹({})", directory.string());
			return false;
		}

		for (const auto& entry : boost::filesystem::directory_iterator{directory}) {
			if (!entry.is_regular_file()) continue;
			const auto& file = entry.path();
			if (const auto extension = file.extension(); extension == ".enabled")
				scripts.push_back({file, true});
			else if (extension == ".disabled")
				scripts.push_back({file, false});
		}

		return !scripts.empty();
	}

	enum class UserCommandType {
		Exit   = 0,
		Toggle = 1
	};

	int GetSelector() noexcept {
		int selector{-1};
		std::cin >> selector;
		return selector;
	}

	UserCommandType GetUserCommand(const int selector) noexcept {
		return selector < 0 ? UserCommandType::Exit : UserCommandType::Toggle;
	}

	void ApplyUserCommand(const std::span<ScriptInfo> scripts, const int selector, bool& exit) noexcept {
		switch (GetUserCommand(selector)) {
		case UserCommandType::Exit:
			exit = true;
			return;

		case UserCommandType::Toggle:
			if (static_cast<std::size_t>(selector) >= scripts.size()) {
				spdlog::warn("不存在用户指定的序号({})，已忽略用户指令", selector);
				return;
			}

			auto& file = scripts[selector];
			if (file.TryFlip()) {
				spdlog::info("成功修改文件({})的状态", file.Path.string());
				return;
			}
			spdlog::error("无法修改文件({})的状态，程序即将退出", file.Path.string());
			exit = true;
			break;
		}
	}

	void UserLoop(ScriptList& scripts) noexcept {
		ScopeNotifier notifier{"用户交互循环"};
		bool exit{false};
		do {
			spdlog::info("脚本状态列表：");
			for (std::size_t index = 0; index < scripts.size(); ++index)
				spdlog::info("{}: {}", index, ToString(scripts[index]));
			spdlog::info("----------------------------------------");
			spdlog::info("输入负数退出程序，输入脚本序号切换开关状态：");
			ApplyUserCommand(scripts, GetSelector(), exit);
		} 
		while (!exit);
	}
}

/// 观测 std::cin 的行为，看按下常见的键后 std::cin 的状态
void TestLoop() {
	std::array<std::istream::char_type, 64> buffer{};
	std::ifstream::sync_with_stdio(false);
	auto* const cin_buffer = std::cin.rdbuf();
	while (true) {
		std::cin.sync();
		auto count = cin_buffer->in_avail();
		if (count < 2) {
			std::this_thread::sleep_for(std::chrono::milliseconds{500});
			continue;
		}

		buffer.fill(0);
		const auto distance = std::min(count, std::streamsize{60});
		std::cin.read(buffer.data(), distance);
		std::span text{buffer.data(), static_cast<std::size_t>(distance)};
		std::cout << fmt::format("buffer> {}\n", std::string{text.data()});
		std::ranges::fill(text, 0);
	}
}

int main() {
	TestLoop();

	if (!InitializeGlobalLogger("aam")) return 1;

	const path_t autorun_dir{"./autorun.d"};
	ScriptList scripts{};
	if (!AcquireScriptList(autorun_dir, scripts)) {
		spdlog::info("未找到脚本文件，正在退出");
		return 0;
	}
	UserLoop(scripts);
	return 0;
}
