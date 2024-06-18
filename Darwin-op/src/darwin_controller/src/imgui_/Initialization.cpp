#include <iostream>
#include <LGui/LGui.h>
#include <SDL2/SDL.h>

_L_GUI_BEGIN

LGui::LGui()
{
    static bool ExitFlag=false;
    if(ExitFlag)
    {
        std::cout<<"Another GUI has been created! Only one GUI can be used in the same process! QUIT!!!"<<std::endl;
        exit(1);
    }
    ExitFlag = true;
    this->RunningFlag = false;
    this->Settings.ClearColor = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
    this->Settings.WindowSize = {1280, 720};
}

void LGui::init()
{
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0)
    {
        printf("Error: %s\n", SDL_GetError());
        return;
    }
    // From 2.0.18: Enable native IME.
    #ifdef SDL_HINT_IME_SHOW_UI
        SDL_SetHint(SDL_HINT_IME_SHOW_UI, "1");
    #endif

    // Create SDL Window
    auto &Width  = this->Settings.WindowSize.Width;
    auto &Height = this->Settings.WindowSize.Height;
    SDL_WindowFlags WindowFlags = (SDL_WindowFlags)(SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    Window = SDL_CreateWindow("LGui", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, Width, Height, WindowFlags);

    // Setup SDL_Renderer instance
    Renderer = SDL_CreateRenderer(Window, -1, SDL_RENDERER_PRESENTVSYNC | SDL_RENDERER_ACCELERATED);
    if (Renderer == NULL)
    {
        SDL_Log("Error creating SDL_Renderer!");
        return;
    }
    SDL_RendererInfo info;
    SDL_GetRendererInfo(Renderer, &info);
    SDL_Log("Current SDL_Renderer: %s", info.name);

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    // (void)io;
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    // io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;

    ImPlot::CreateContext();

    // Setup Dear ImGui style
    // ImGui::StyleColorsDark();
    ImGui::StyleColorsClassic();

    // Setup Platform/Renderer backends
    ImGui_ImplSDL2_InitForSDLRenderer(Window, Renderer);
    ImGui_ImplSDLRenderer_Init(Renderer);
}

_L_GUI_END