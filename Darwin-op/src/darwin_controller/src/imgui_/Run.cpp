#include <LGui/LGui.h>
#include <SDL2/SDL.h>
#include <iostream>

_L_GUI_BEGIN

void LGui::run()
{
    this->RunningFlag = true;
    while(this->RunningFlag)
    {
        SDL_Event event;
        this->PressKey = 0;
        while (SDL_PollEvent(&event))
        {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT)
                RunningFlag = false;
            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE && event.window.windowID == SDL_GetWindowID(Window))
                RunningFlag = false;
            if (event.type == SDL_KEYDOWN)
            {
                this->PressKey = event.key.keysym.sym;
            }
            if(this->PressKey == ' ') RunningFlag = false;
        }
        // Start the Dear ImGui frame
        ImGui_ImplSDLRenderer_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        ImGui::Begin("LGui");      
        ImGui::Text("Please switch to English Input Method! Press SPACE to quit."); // Display some text (you can use a format strings too)
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / ImGui::GetIO().Framerate, ImGui::GetIO().Framerate);
        ImGui::End();

        // Other gui codes
        this->updateFrame();

        // Rendering
        auto &clear_color = this->Settings.ClearColor;
        ImGui::Render();
        SDL_SetRenderDrawColor(Renderer, (Uint8)(clear_color.x * 255), (Uint8)(clear_color.y * 255), (Uint8)(clear_color.z * 255), (Uint8)(clear_color.w * 255));
        SDL_RenderClear(Renderer);
        ImGui_ImplSDLRenderer_RenderDrawData(ImGui::GetDrawData());
        SDL_RenderPresent(Renderer);
    }

    // Cleanup
    ImGui_ImplSDLRenderer_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    SDL_DestroyRenderer(Renderer);
    SDL_DestroyWindow(Window);
    SDL_Quit();
}

_L_GUI_END