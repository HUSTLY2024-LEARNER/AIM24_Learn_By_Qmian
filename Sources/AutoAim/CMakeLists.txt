project(LangYa.AutoAim)

AddCXXModule(
	NAME "AutoAim"
	NAMESPACE "LangYa"
	LINKS
		fmt::fmt
		spdlog::spdlog
		LangYa::ArmorDetectors
		Ceres::ceres
		Sophus::Sophus
		Cango_ByteCommunication::BoostImplementations
		Cango::CommonUtils
		Cango::GalaxySDK
)
