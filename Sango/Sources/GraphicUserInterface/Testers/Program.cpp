#include <imgui.h>
#include <imgui_impl_sdl2.h>
#include <imgui_impl_sdlrenderer2.h>
#include <Cango/ByteCommunication.hpp>
#include <LangYa/AutoAim.hpp>
#include <numbers>

#ifdef _WINDOWS
#include <imgui_impl_win32.h>
#endif

#include <SDL.h>
#undef main

using namespace LangYa;

/// @brief 平滑过程
///	@param begin 开始值
///	@param end 结束值
///	@param progress 过程进度 [0.0f, 1.0f]
float SmoothProgress(const float begin, const float end, const float progress)
{
	constexpr auto pi = std::numbers::pi_v<float>;
	const auto k1 = std::acos(1.0f - 2.0f * progress) / pi;
	const auto k2 = 0.5f * (1.0f - std::cos(progress * pi));
	const auto k = k1 * (1.0f - progress) + k2 * progress;
	return begin + (end - begin) * k;
}

float ExpectProgress(
	const std::chrono::steady_clock::time_point& begin,
	const std::chrono::milliseconds& duration,
	const std::chrono::steady_clock::time_point& current)
{
	if (current < begin) return 0.0f;
	const auto interval = std::chrono::duration_cast<std::chrono::milliseconds>(current - begin);
	if (interval > duration) return 1.0f;
	return static_cast<float>(interval.count()) / static_cast<float>(duration.count());
}

namespace SDL
{
	const char* GetLastError()
	{
		static const auto* empty = "";
		const auto* message = SDL_GetError();
		if (message == nullptr)
		{
			spdlog::warn("SDL> got empty error message");
			return empty;
		}
		return message;
	}

	bool Initialize()
	{
		if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0)
		{
			spdlog::error("InitializeSDL> SDL_Init exception: {}", SDL_GetError());
			return false;
		}

		if (!SDL_SetHint(SDL_HINT_IME_SHOW_UI, "1"))
		{
			spdlog::error("InitializeSDL> SDL_SetHint exception: {}", SDL_GetError());
			return false;
		}

		return true;
	}

	void DrawBackground(SDL_Renderer& renderer)
	{
		if (SDL_RenderClear(&renderer) == 0) return;

		spdlog::error("SDL> cannot draw background: {}", GetLastError());
	}

	void RenderSetDrawColor(SDL_Renderer& renderer, const ImVec4& color)
	{
		if (SDL_SetRenderDrawColor(&renderer,
		                           static_cast<Uint8>(color.x * 255),
		                           static_cast<Uint8>(color.y * 255),
		                           static_cast<Uint8>(color.z * 255),
		                           static_cast<Uint8>(color.w * 255)
		) == 0)
			return;

		spdlog::warn("SDL> cannot set renderer draw color: {}", GetLastError());
	}

	void DrawBackground(SDL_Renderer& renderer, const ImVec4& color)
	{
		RenderSetDrawColor(renderer, color);
		DrawBackground(renderer);
	}

	void RenderSetScale(SDL_Renderer& renderer, const ImVec2& scale)
	{
		if (SDL_RenderSetScale(&renderer, scale.x, scale.y) == 0) return;
		spdlog::warn("SDL> cannot set renderer scale: {}", GetLastError());
	}

	void RenderImGui(SDL_Renderer& renderer)
	{
		ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), &renderer);
		SDL_RenderPresent(&renderer);
	}

	bool CheckEventToQuit(const SDL_Event& event)
	{
		if (event.type == SDL_QUIT)
			return true;
		if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE)
			return true;
		return false;
	}

	bool CreateWindow(SDL_Window* & windowHandle, const SDL_WindowFlags windowFlags)
	{
		windowHandle = SDL_CreateWindow(
			"MainWindow",
			SDL_WINDOWPOS_CENTERED,
			SDL_WINDOWPOS_CENTERED,
			1280,
			720,
			windowFlags
		);
		if (windowHandle == nullptr)
		{
			spdlog::error("Main> cannot create window: {}", SDL_GetError());
			return false;
		}
		return true;
	}

	bool CreateRenderer(SDL_Window& window, SDL_Renderer* & rendererHandle)
	{
		rendererHandle = SDL_CreateRenderer(&window, -1, SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);
		if (rendererHandle == nullptr)
		{
			spdlog::error("Main> cannot create renderer: {}", SDL_GetError());
			return false;
		}
		return true;
	}

	bool SetupWindowRender(
		SDL_Window* & windowHandle,
		const SDL_WindowFlags windowFlags,
		SDL_Renderer* & rendererHandle)
	{
		return CreateWindow(windowHandle, windowFlags)
			&& CreateRenderer(*windowHandle, rendererHandle);
	}

	bool SetupImGui(SDL_Window* & windowHandle, SDL_Renderer* & rendererHandle)
	{
		return ImGui_ImplSDL2_InitForSDLRenderer(windowHandle, rendererHandle)
			&& ImGui_ImplSDLRenderer2_Init(rendererHandle);
	}

	void CleanImGui(SDL_Window* & windowHandle, SDL_Renderer* & rendererHandle)
	{
		ImGui_ImplSDLRenderer2_Shutdown();
		ImGui_ImplSDL2_Shutdown();
		ImGui::DestroyContext();

		SDL_DestroyRenderer(rendererHandle);
		SDL_DestroyWindow(windowHandle);
		SDL_Quit();
	}

	void BeginImGuiFrame()
	{
		ImGui_ImplSDLRenderer2_NewFrame();
		ImGui_ImplSDL2_NewFrame();
		ImGui::NewFrame();
	}

	void EndImGuiFrame(SDL_Renderer& renderer, const ImVec2 scale, const ImVec4& clearColor)
	{
		ImGui::Render();
		RenderSetScale(renderer, scale);
		DrawBackground(renderer, clearColor);
		RenderImGui(renderer);
	}
}

namespace ImGui::Sango
{
	void MyBox(ImVec4& clearColor)
	{
		static float f = 0.0f;
		static int counter = 0;

		Begin("Hello, world!"); // Create a window called "Hello, world!" and append into it.

		Text("This is some useful text."); // Display some text (you can use a format strings too)

		SliderFloat("float", &f, 0.0f, 1.0f); // Edit 1 float using a slider from 0.0f to 1.0f
		ColorEdit3("clear color", reinterpret_cast<float*>(&clearColor)); // Edit 3 floats representing a color

		if (Button("Button")) counter++;
		SameLine();
		Text("counter = %d", counter);

		const auto& io = GetIO();
		Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
		End();
	}

	class SmoothEngine final
	{
		float TargetRecorded{0.0f};
		float TargetBegin{0.0f};
		float TargetSmoothed{0.0f};
		std::chrono::steady_clock::time_point ChangeBegin{std::chrono::steady_clock::now()};

	public:
		float Update(const float target, const std::chrono::steady_clock::time_point& currentTime) noexcept
		{
			if (std::abs(target - TargetRecorded) > 1e-5)
			{
				TargetRecorded = target;
				ChangeBegin = currentTime;
				TargetBegin = TargetSmoothed;
			}
			const auto expected = ExpectProgress(
				ChangeBegin,
				std::chrono::milliseconds{500},
				currentTime
			);
			return TargetSmoothed = SmoothProgress(TargetBegin, target, expected);
		}
	};

	class SmoothProgressBar final
	{
		SmoothEngine Engine{};

	public:
		float operator()(const float target, const std::chrono::steady_clock::time_point& currentTime)
		{
			const auto progress = Engine.Update(target, currentTime);
			ProgressBar(progress);
			return progress;
		}
	};

	class Notify final
	{
	public:
		bool IsVisible{true};
		std::string Message{"notify"};

		bool operator()()
		{
			if (!IsVisible) return false;
			IsVisible = !Button(Message.data());
			return IsVisible;
		}
	};

	class NotifyList final
	{
		std::list<Notify> List{};

	public:
		void AddNotify(const Notify& notify)
		{
			List.emplace_back(notify);
		}

		void operator()()
		{
			auto iterator = List.begin();
			while (iterator != List.end())
			{
				if (!(*iterator)()) iterator = List.erase(iterator);
				else ++iterator;
			}
		}
	};
}

SDL_WindowFlags operator |(const SDL_WindowFlags& lhs, const SDL_WindowFlags& rhs)
{
	return static_cast<SDL_WindowFlags>(static_cast<int>(lhs) | static_cast<int>(rhs));
}

using namespace std::chrono_literals;

int main(const int argc, const char* argv[])
{
	if (!InitializeGlobalLogger("gui")) return -1;
	if (!SDL::Initialize()) return -1;

#ifdef _WINDOWS
	ImGui_ImplWin32_EnableDpiAwareness();
#endif

	const auto window_flags = SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI;
	SDL_Window* window_handle{nullptr};
	SDL_Renderer* renderer_handle{nullptr};
	if (!SDL::SetupWindowRender(window_handle, window_flags, renderer_handle)) return -1;

	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();
	io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
	ImGui::StyleColorsDark();
	SDL::SetupImGui(window_handle, renderer_handle);
	(void)io.Fonts->AddFontFromFileTTF("FiraCode-Retina.ttf", 24.0f, nullptr, io.Fonts->GetGlyphRangesChineseFull());

	constexpr auto clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

	ImGui::Sango::NotifyList notify_list{};

	bool done = false;
	while (!done)
	{
		SDL_Event event;
		while (SDL_PollEvent(&event))
		{
			ImGui_ImplSDL2_ProcessEvent(&event);
			done = SDL::CheckEventToQuit(event);
		}
		if (done) break;

		SDL::BeginImGuiFrame();

		//ImGui::Sango::MyBox(clear_color);

		ImGui::Begin("animation");

		const auto current_time = std::chrono::steady_clock::now();

		static int id = 0;

		static float height = 0.0f;
		ImGui::DragFloat("height", &height, 1, 28, 1000.0f);
		static ImGui::Sango::SmoothEngine engine{};
		const auto smooth = engine.Update(height, current_time);
		if (ImGui::Button("NewNotify", ImVec2{0.0f, smooth}))
			notify_list.AddNotify(ImGui::Sango::Notify{true, fmt::format("notify{}", id++)});

		ImGui::Button("text1\n text2");

		notify_list();

		ImGui::End();

		SDL::EndImGuiFrame(*renderer_handle, io.DisplayFramebufferScale, clear_color);
	}

	SDL::CleanImGui(window_handle, renderer_handle);
	spdlog::drop_all();

	return 0;
}
