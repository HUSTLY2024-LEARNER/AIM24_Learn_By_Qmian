#include <iostream>
#include <ranges>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string.hpp>
#include <Cango/TaskDesign/FormattableObject.hpp>
#include <spdlog/spdlog.h>

using namespace boost;

using path_t = boost::filesystem::path;

struct TargetFile
{
	path_t Path{};
	std::vector<std::string> Tags{};

	std::ostream& Format(std::ostream& stream) const
	{
		std::ranges::for_each(Tags, [&stream](const auto& tag)
		{
			stream << '[' << tag << "] ";
		});
		return stream << Path;
	}

	[[nodiscard]] bool CopyTo(const path_t& destination) const
	{
		system::error_code result{};
		copy(Path, destination, filesystem::copy_options::overwrite_existing, result);

		if (!result.failed()) return true;
		spdlog::error("failed to copy {} to {}: {}", Path.string(), destination.string(), result.what());
		return false;
	}
};

using TargetList = std::vector<TargetFile>;

void ScanDirectory(const path_t& directoryPath, const std::string& alternativePrefix, TargetList& exeList)
{
	for (const auto& entry : filesystem::directory_iterator{directoryPath})
	{
		if (!entry.is_regular_file()) continue;

		const auto& file = entry.path();
		const auto filename = file.filename().string();

		if (!filename.starts_with(alternativePrefix)) continue;

		std::vector<std::string> components{};
		split(components, filename, is_any_of("_ "));

		if (components.size() <= 1) continue;

		TargetFile auto_aim_exe{file};
		std::ranges::copy(components | std::views::drop(1), std::back_inserter(auto_aim_exe.Tags));
		exeList.emplace_back(std::move(auto_aim_exe));
	}
}

int main()
{
	const std::string name{"AutoAimManager"};
	const path_t autoaim_dir{"./autoaim.d"};
	const path_t config_path{"./autoaim.d/config.ini"};

	if (!exists(autoaim_dir))
	{
		spdlog::error(
			"{}> cannot find directory({})",
			name,
			autoaim_dir.string());
		return -1;
	}

	spdlog::debug("{}> reading config from {}", name, config_path.string());

	const std::string last_target_filename_key{"LastTargetFilename"};
	std::string last_target_filename{};
	property_tree::ptree tree{};
	if (!DefaultLoad(tree, config_path)
		|| !DefaultConfigure(tree, last_target_filename_key, last_target_filename))
	{
		spdlog::warn("{}> failed to load config, preparing empty config file", name);
		tree.put(last_target_filename_key, last_target_filename);
		if (!DefaultSave(tree, config_path)) return -1;
	}

	TargetList auto_aim_exe_list{};
	ScanDirectory(autoaim_dir, "AutoAim", auto_aim_exe_list);

	if (auto_aim_exe_list.empty())
	{
		spdlog::info("{}> no auto aim exe to manage, exiting", name);
		return 0;
	}

	int current_selected{-1};
	int index{0};
	std::ranges::for_each(
		auto_aim_exe_list,
		[&index, &current_selected, last_target_filename, name](const TargetFile& auto_aim_exe)
		{
			if (current_selected == -1 && auto_aim_exe.Path.filename().string() == last_target_filename)
				current_selected = index;

			spdlog::info(
				"{}> {}: ({}) {}",
				name,
				index,
				index == current_selected ? "x" : " ",
				Cango::Format(auto_aim_exe)
			);
			index++;
		});

	spdlog::info("{}> ----------------------------------------", name);
	spdlog::info("{}> enter a negative number to exit", name);
	spdlog::info("{}> enter the number of the exe you want to select", name);

	int selector{-1};
	while (true)
	{
		std::cout << "> ";
		std::cin >> selector;
		if (selector < index) break;
		spdlog::info("{}> invalid exe number, please retry", name);
	}

	if (selector < 0)
	{
		spdlog::info("{}> exiting", name);
		return 0;
	}

	const path_t destination_path{"./autoaim.d/LangYa_AutoAim_Main"};
	const auto& selected_exe = auto_aim_exe_list[selector];
	if (!selected_exe.CopyTo(destination_path))
	{
		spdlog::error("{}> failed to copy selected exe", name);
		return -1;
	}

	spdlog::info("{}> copied {} to {}", name, selected_exe.Path.string(), destination_path.string());
	tree.put(last_target_filename_key, selected_exe.Path.filename().string());

	if (!DefaultSave(tree, config_path))
	{
		spdlog::error("{}> failed to save config", name);
		return -1;
	}

	return 0;
}
