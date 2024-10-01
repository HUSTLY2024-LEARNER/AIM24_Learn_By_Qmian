project(LangYa.Common)

AddCXXModule(
	NAME "Common"
	NAMESPACE "LangYa"
	VERSION 1.0.2.0
	LINKS
		spdlog::spdlog
		Boost::system
		Boost::json
		Boost::filesystem
		Boost::program_options
		Taskflow::Taskflow
		TBB::tbb
		${OpenCV_LIBRARIES}
		Cango::TaskDesign
)

