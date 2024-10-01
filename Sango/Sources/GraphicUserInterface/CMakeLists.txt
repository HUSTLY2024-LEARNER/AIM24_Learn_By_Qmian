project(Sango.GraphicUserInterface)

file(COPY Assets/FiraCode-Retina.ttf DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

AddCXXModule(
	NAME GraphicUserInterface
	NAMESPACE Sango
	LINKS
		LangYa::Common
		imgui::imgui
		SDL2::SDL2
		Cango::ByteCommunication
		LangYa::AutoAim
)
