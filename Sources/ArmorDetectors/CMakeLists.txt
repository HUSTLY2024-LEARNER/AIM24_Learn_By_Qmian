project(LangYa.ArmorDetectors)

AddCXXModule(
	NAME "ArmorDetectors"
	NAMESPACE "LangYa"
	LINKS
		LangYa::Common
		Eigen3::Eigen
		openvino::runtime
)
